import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def rpc_settings(context):
    import yaml
    with open(LaunchConfiguration('canrpc_config').perform(context), encoding='utf-8') as source:
        profile = yaml.safe_load(source)
    rpc = profile['rpc']
    response_sec = float(rpc['response_timeout_ms']) / 1000.0

    def endpoint_seconds(name):
        endpoint = next((item for item in profile.get('rpc_endpoints', [])
                         if item['name'] == name), None)
        policy = next((node.get('rpc', {}) for node in profile['nodes'].values()
                       if endpoint and node['id'] == endpoint['destination']), {})
        return float(policy.get('response_timeout_ms', rpc['response_timeout_ms'])) / 1000.0

    pump_sec = endpoint_seconds('pump_set')
    target_sec = endpoint_seconds(
        context.launch_configurations.get('arm_target_endpoint', 'arm_target'))
    initialization_sec = max(response_sec + 0.5,
                             target_sec + endpoint_seconds('arm_ready') + 1.0)
    application_sec = max(initialization_sec, pump_sec + 0.5)
    return [
        SetLaunchConfiguration('canrpc_service', rpc.get('ros_service', 'canrpc/call')),
        SetLaunchConfiguration('rpc_timeout_sec', str(pump_sec + 0.5)),
        SetLaunchConfiguration('target_rpc_timeout_sec', str(target_sec + 0.05)),
        SetLaunchConfiguration('target_period_ms', str(
            profile.get('application', {}).get('arm_control', {}).get('target_period_ms', 10))),
        SetLaunchConfiguration('web_request_timeout_ms', str(
            int((application_sec + 1.0) * 1000))),
        SetLaunchConfiguration('initialization_rpc_timeout_sec', str(initialization_sec)),
        SetLaunchConfiguration('application_service_timeout_sec', str(application_sec + 0.5)),
        SetLaunchConfiguration(
            'sequence_service_timeout_sec', str(max(3.0, application_sec + 0.5))),
    ]


def generate_launch_description():
    pump_config = os.path.join(
        get_package_share_directory('catchrobo2026_pump'), 'config', 'pump.yaml')
    festival_config = os.path.join(
        get_package_share_directory('catchrobo2026_pump'),
        'config', 'festival.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('canrpc_config', default_value=os.path.join(
            get_package_share_directory('canrpc_bridge'), 'config', 'gakusai.yaml')),
        OpaqueFunction(function=rpc_settings),

        DeclareLaunchArgument('festival_config', default_value=festival_config),
        DeclareLaunchArgument(
            'pump_config',
            default_value=pump_config,
            description='Pump controller parameter file'
        ),
        Node(
            package='catchrobo2026_pump',
            executable='pump_controller_node',
            name='pump_controller_node',
            parameters=[LaunchConfiguration('pump_config'),
                        LaunchConfiguration('festival_config'), {
                            'canrpc_service': LaunchConfiguration('canrpc_service'),
                            'rpc_timeout_sec': ParameterValue(
                                LaunchConfiguration('rpc_timeout_sec'), value_type=float),
                        }],
            output='screen'
        )
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def package_path(package, *parts):
    return PathJoinSubstitution([get_package_share_directory(package), *parts])


def rpc_settings(context):
    import yaml
    with open(LaunchConfiguration('canrpc_config').perform(context), encoding='utf-8') as source:
        profile = yaml.safe_load(source)
    pressure_routes = [route for route in profile.get('routes', []) if route['name'] == 'pressure']
    if any(type(route.get('enabled', True)) is not bool for route in pressure_routes):
        raise ValueError('pressure route.enabled must be boolean')
    pressure_enabled = any(route.get('enabled', True) for route in pressure_routes)
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
    motor = profile.get('application', {}).get('motor_initialization', {})
    initialization_sec = max(response_sec + 0.5,
                             float(motor.get('initialization_timeout_ms', 0)) / 1000.0 +
                             target_sec + endpoint_seconds('arm_initialize') +
                             endpoint_seconds('arm_ready') + 1.0)
    application_sec = max(initialization_sec, pump_sec + 0.5)
    return [
        SetLaunchConfiguration('rebase_max_delta_rad', str(motor.get('rebase_max_delta_rad', 0.2))),
        SetLaunchConfiguration('pressure_enabled', str(pressure_enabled).lower()),
        SetLaunchConfiguration('canrpc_service', rpc.get('ros_service', 'canrpc/call')),
        SetLaunchConfiguration('rpc_timeout_sec', str(pump_sec + 0.5)),
        SetLaunchConfiguration('target_rpc_timeout_sec', str(target_sec + 0.05)),
        SetLaunchConfiguration('trajectory_cleanup_timeout_sec', str(target_sec + 1.5)),
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
    return LaunchDescription([
        DeclareLaunchArgument('canrpc_config', default_value=os.path.join(
            get_package_share_directory('canrpc_bridge'), 'config', 'gakusai.yaml')),
        DeclareLaunchArgument('arm_target_endpoint', default_value='arm_target'),
        DeclareLaunchArgument('canrpc_status_topic', default_value='canrpc/status'),
        DeclareLaunchArgument('bridge_status_timeout_sec', default_value='2.5'),
        DeclareLaunchArgument('trajectory_dynamics_config', default_value=os.path.join(
            get_package_share_directory('path_planner'), 'config', 'robot_dynamics.yaml')),
        DeclareLaunchArgument('trajectory_pipeline_config', default_value=os.path.join(
            get_package_share_directory('path_planner'), 'config', 'gakusai_trajectory.yaml')),
        OpaqueFunction(function=rpc_settings),

        DeclareLaunchArgument('team', default_value='red', choices=['red', 'blue']),
        DeclareLaunchArgument(
            'sequence_file', default_value=package_path(
                'catchrobo2026_sequence', 'config', 'sequences.yaml')),
        DeclareLaunchArgument('festival_config', default_value=PathJoinSubstitution([
            get_package_share_directory('catchrobo2026_pump'),
            'config', 'festival.yaml'])),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('joy_source', default_value='web', choices=['web']),
        DeclareLaunchArgument('manual_config', default_value=package_path(
            'catchrobo2026_ui', 'config', 'manual.yaml')),
        DeclareLaunchArgument(
            'pump_config', default_value=package_path(
                'catchrobo2026_pump', 'config', 'pump.yaml')),
        DeclareLaunchArgument(
            'joint_feedback_config', default_value=package_path(
                'nav_director', 'config', 'joint_feedback.yaml')),
        DeclareLaunchArgument(
            'manual_velocity_config', default_value=package_path(
                'catchrobo2026_hand_operated', 'config', 'manual_velocity.yaml')),
        DeclareLaunchArgument('ipc_socket', default_value='/tmp/catchrobo2026-control.sock'),
        DeclareLaunchArgument('listen', default_value='0.0.0.0:8080'),

        # Include Launches
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_path(
                'catchrobo2026_sequence', 'launch', 'sequence.launch.py')),
            launch_arguments={
                'team': LaunchConfiguration('team'),
                'sequence_file': LaunchConfiguration('sequence_file'),
                'service_timeout_sec': LaunchConfiguration('sequence_service_timeout_sec'),
                'festival_config': LaunchConfiguration('festival_config'),
                'debug': LaunchConfiguration('debug'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_path(
                'catchrobo2026_ui', 'launch', 'ui.launch.py')),
            launch_arguments={
                'ipc_socket': LaunchConfiguration('ipc_socket'),
                'listen': LaunchConfiguration('listen'),
                'sequence_enabled': 'true',
                'pressure_enabled': LaunchConfiguration('pressure_enabled'),
                'request_timeout_ms': LaunchConfiguration('web_request_timeout_ms'),
                'trajectory_cleanup_enabled': 'true',
                'trajectory_cleanup_timeout_sec': LaunchConfiguration(
                    'trajectory_cleanup_timeout_sec'),
                'initialization_timeout_sec': LaunchConfiguration(
                    'application_service_timeout_sec'),
                'manual_service_timeout_sec': LaunchConfiguration(
                    'application_service_timeout_sec'),
                'manual_config': LaunchConfiguration('manual_config'),
                'pump_config': LaunchConfiguration('pump_config'),
                'festival_config': LaunchConfiguration('festival_config'),
                'manual_velocity_config': LaunchConfiguration('manual_velocity_config'),
                'joy_source': LaunchConfiguration('joy_source'),
                'debug': LaunchConfiguration('debug'),
            }.items()),

        # Existing Nodes
        Node(package='nav_director', executable='path_generator_3d',
             parameters=[LaunchConfiguration('joint_feedback_config')], output='screen'),
        Node(package='nav_director', executable='path_follower_node',
             parameters=[LaunchConfiguration('joint_feedback_config'), {
                 'lease_release_timeout_sec': ParameterValue(
                     LaunchConfiguration('trajectory_cleanup_timeout_sec'), value_type=float),
             }], output='screen'),
        Node(
            package='catchrobo2026_state_machine', executable='arm_target_client',
            name='arm_target_client', output='screen', parameters=[{
                'canrpc_service': LaunchConfiguration('canrpc_service'),
                'arm_target_endpoint': LaunchConfiguration('arm_target_endpoint'),
                'rebase_max_delta_rad': ParameterValue(
                    LaunchConfiguration('rebase_max_delta_rad'), value_type=float),
                'canrpc_status_topic': LaunchConfiguration('canrpc_status_topic'),
                'bridge_status_timeout_sec': ParameterValue(
                    LaunchConfiguration('bridge_status_timeout_sec'), value_type=float),
                'trajectory_dynamics_config': LaunchConfiguration('trajectory_dynamics_config'),
                'trajectory_pipeline_config': LaunchConfiguration('trajectory_pipeline_config'),
                'target_period_ms': ParameterValue(
                    LaunchConfiguration('target_period_ms'), value_type=int),
                'rpc_timeout_sec': ParameterValue(
                    LaunchConfiguration('target_rpc_timeout_sec'), value_type=float),
            }]),
        Node(package='catchrobo2026_state_machine', executable='state_machine_node',
             name='state_machine_node', parameters=[{
                 'canrpc_service': LaunchConfiguration('canrpc_service'),
                 'rpc_timeout_sec': ParameterValue(
                     LaunchConfiguration('initialization_rpc_timeout_sec'), value_type=float),
                'arm_initialize_endpoint': 'arm_initialize',
             }], output='screen'),
        Node(package='catchrobo2026_pump', executable='pump_controller_node',
             parameters=[LaunchConfiguration('pump_config'),
                         LaunchConfiguration('festival_config'), {
                            'pressure_enabled': ParameterValue(
                                LaunchConfiguration('pressure_enabled'), value_type=bool),
                            'canrpc_service': LaunchConfiguration('canrpc_service'),
                            'rpc_timeout_sec': ParameterValue(
                                LaunchConfiguration('rpc_timeout_sec'), value_type=float),
                        }], output='screen'),

        # ==================================
        # main.launch.py から追加したノード
        # ==================================
        Node(
            package='catchrobo2026_hand_operated',
            executable='joy_controller_node',
            name='joy_controller_node',
            parameters=[LaunchConfiguration('joint_feedback_config'),
                        LaunchConfiguration('manual_velocity_config'),
                        LaunchConfiguration('festival_config'), {
                            'debug': ParameterValue(
                                LaunchConfiguration('debug'), value_type=bool),
                            'service_timeout_sec': ParameterValue(
                                LaunchConfiguration('application_service_timeout_sec'),
                                value_type=float),
                            'rebase_max_delta_rad': ParameterValue(
                                LaunchConfiguration('rebase_max_delta_rad'), value_type=float),
                            'manual_velocity_config': ParameterValue(
                                LaunchConfiguration('manual_velocity_config'), value_type=str),
                        }],
            output='screen'
        ),
        Node(
            package='nav_director',
            executable='current_kinematics_visualizer',
            name='current_kinematics_visualizer',
            output='screen'
        ),
    ])

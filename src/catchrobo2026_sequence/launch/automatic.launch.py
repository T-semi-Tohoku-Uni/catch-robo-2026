import importlib.util
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def _load_restart_coordinator():
    path = os.path.join(os.path.dirname(__file__), 'runtime_restart.py')
    spec = importlib.util.spec_from_file_location('catchrobo_runtime_restart', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RuntimeRestartCoordinator


def package_path(package, *parts):
    return PathJoinSubstitution([get_package_share_directory(package), *parts])


def generate_launch_description():
    reset_contract = {'enabled': False}
    arguments = [
        DeclareLaunchArgument('team', choices=['red', 'blue']),
        DeclareLaunchArgument(
            'sequence_file', default_value=package_path(
                'catchrobo2026_sequence', 'config', 'sequences.yaml')),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('joy_source', default_value='web', choices=['web', 'local']),
        DeclareLaunchArgument('manual_config', default_value=package_path(
            'catchrobo2026_ui', 'config', 'manual.yaml')),
        DeclareLaunchArgument(
            'queue_config', default_value=package_path(
                'catchrobo2026_ui', 'config', 'queue.yaml')),
        DeclareLaunchArgument(
            'pump_config', default_value=package_path(
                'catchrobo2026_pump', 'config', 'pump.yaml')),
        DeclareLaunchArgument(
            'joint_feedback_config', default_value=package_path(
                'nav_director', 'config', 'joint_feedback.yaml')),
        DeclareLaunchArgument('ipc_socket', default_value='/tmp/catchrobo2026-control.sock'),
        DeclareLaunchArgument('listen', default_value='0.0.0.0:8080'),
        DeclareLaunchArgument('runtime_supervised', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('runtime_reset_enabled', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'runtime_state_file',
            default_value=''),
    ]

    def runtime_actions():
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_path(
                'catchrobo2026_sequence', 'launch', 'sequence.launch.py')),
            launch_arguments={
                'team': LaunchConfiguration('team'),
                'sequence_file': LaunchConfiguration('sequence_file'),
                'debug': LaunchConfiguration('debug'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_path(
                'catchrobo2026_ui', 'launch', 'ui.launch.py')),
            launch_arguments={
                'ipc_socket': LaunchConfiguration('ipc_socket'),
                'listen': LaunchConfiguration('listen'),
                'sequence_enabled': 'true',
                'queue_config': LaunchConfiguration('queue_config'),
                'manual_config': LaunchConfiguration('manual_config'),
                'joy_source': LaunchConfiguration('joy_source'),
                'runtime_reset_enabled': LaunchConfiguration('runtime_reset_enabled'),
            }.items()),
            
        # Existing Nodes
        Node(package='nav_director', executable='path_generator_3d',
             parameters=[LaunchConfiguration('joint_feedback_config')], output='screen'),
        Node(package='nav_director', executable='path_follower_node',
             parameters=[LaunchConfiguration('joint_feedback_config')], output='screen'),
        Node(package='catchrobo2026_state_machine', executable='state_machine_node',
             name='state_machine_node', output='screen', parameters=[{
                 'state_file': LaunchConfiguration('runtime_state_file'),
                 'require_state_file': True,
             }] if reset_contract['enabled'] else []),
        Node(package='catchrobo2026_pump', executable='pump_controller_node',
             parameters=[LaunchConfiguration('pump_config')], output='screen'),
        Node(package='catchrobo2026_endeffector', executable='endeffector_state_node',
             output='screen'),

        # ==================================
        # main.launch.py から追加したノード
        # ==================================
        Node(
            package='joy', 
            executable='joy_node', 
            name='joy_node', 
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('joy_source'), "' == 'local'"
            ])),
            output='screen'
        ),
        Node(
            package='catchrobo2026_hand_operated', 
            executable='joy_controller_node', 
            name='joy_controller_node', 
            parameters=[LaunchConfiguration('joint_feedback_config')] + ([{
                'require_current_joints_on_start': True,
            }] if reset_contract['enabled'] else []),
            output='screen'
        ),
        Node(
            package='nav_director', 
            executable='current_kinematics_visualizer', 
            name='current_kinematics_visualizer', 
            output='screen'
        )]

    def configure_runtime(context):
        supervised = LaunchConfiguration('runtime_supervised').perform(context) == 'true'
        reset_enabled = LaunchConfiguration('runtime_reset_enabled').perform(context) == 'true'
        if supervised and not reset_enabled:
            raise RuntimeError('runtime_supervised=true requires runtime_reset_enabled=true')
        reset_contract['enabled'] = reset_enabled
        state_file = LaunchConfiguration('runtime_state_file').perform(context)
        runtime_directory = ''
        if supervised:
            runtime_directory = tempfile.mkdtemp(prefix='catchrobo-runtime-')
        if supervised and not state_file:
            state_file = os.path.join(runtime_directory, 'init-state')
            context.launch_configurations['runtime_state_file'] = state_file
        if reset_enabled and not state_file:
            raise RuntimeError('runtime_reset_enabled=true requires runtime_state_file')
        if supervised and reset_enabled and not os.path.exists(state_file):
            fd = os.open(state_file, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, 'w', encoding='ascii') as stream:
                stream.write('0\n')
                stream.flush()
                os.fsync(stream.fileno())
        ack_socket = os.path.join(runtime_directory, 'supervisor.sock') if supervised else ''
        joy_local = LaunchConfiguration('joy_source').perform(context) == 'local'
        coordinator = _load_restart_coordinator()(
            runtime_actions, enabled=supervised, ack_socket=ack_socket,
            expected_starts=11 if joy_local else 10,
            supervisor_arguments=[
                '--state-file', LaunchConfiguration('runtime_state_file'),
                '--ack-socket', ack_socket,
            ] if supervised else [], session_directory=runtime_directory)
        return coordinator.actions()

    return LaunchDescription(arguments + [OpaqueFunction(function=configure_runtime)])

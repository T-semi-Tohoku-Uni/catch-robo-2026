import importlib.util
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import StateTransition


def is_can_bridge(action):
    return isinstance(action, LifecycleNode) and action.node_name == '/nhk2026_canbridge'


def load_restart_coordinator():
    path = Path(get_package_share_directory('catchrobo2026_sequence')) / \
        'launch/runtime_restart.py'
    spec = importlib.util.spec_from_file_location('catchrobo_runtime_restart', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RuntimeRestartCoordinator


def generate_launch_description():
    workspace = Path(__file__).resolve().parent.parent
    arguments = [
        DeclareLaunchArgument('team', choices=['red', 'blue']),
        DeclareLaunchArgument(
            'sequence_file', default_value=str(
                workspace / 'src/catchrobo2026_sequence/config/sequences.yaml')),
        DeclareLaunchArgument(
            'queue_config', default_value=str(
                workspace / 'src/catchrobo2026_ui/config/queue.yaml')),
        DeclareLaunchArgument(
            'manual_config', default_value=str(
                workspace / 'src/catchrobo2026_ui/config/manual.yaml')),
        DeclareLaunchArgument('joy_source', default_value='web', choices=['web', 'local']),
        DeclareLaunchArgument(
            'pump_config', default_value=str(
                workspace / 'src/catchrobo2026_pump/config/pump.yaml')),
        DeclareLaunchArgument(
            'joint_feedback_config', default_value=str(
                workspace / 'src/nav_director/config/joint_feedback.yaml')),
        DeclareLaunchArgument('debug', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('listen', default_value='0.0.0.0:8080'),
        DeclareLaunchArgument('ipc_socket', default_value='/tmp/catchrobo2026-control.sock'),
        DeclareLaunchArgument('non_blocking', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'runtime_state_file',
            default_value=''),
    ]

    def configure_runtime(context):
        runtime_directory = tempfile.mkdtemp(prefix='catchrobo-production-')
        state_file = LaunchConfiguration('runtime_state_file').perform(context)
        if not state_file:
            state_file = str(Path(runtime_directory) / 'init-state')
            context.launch_configurations['runtime_state_file'] = state_file
        if not os.path.exists(state_file):
            fd = os.open(state_file, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, 'w', encoding='ascii') as stream:
                stream.write('0\n')
                stream.flush()
                os.fsync(stream.fileno())

        def runtime_actions():
            automatic = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(
                    Path(get_package_share_directory('catchrobo2026_sequence')) /
                    'launch/automatic.launch.py')),
                launch_arguments={**{name: LaunchConfiguration(name) for name in (
                    'team', 'sequence_file', 'queue_config', 'pump_config',
                    'joint_feedback_config', 'debug', 'listen', 'ipc_socket',
                    'manual_config', 'joy_source', 'runtime_state_file')},
                    'runtime_supervised': 'false',
                    'runtime_reset_enabled': 'true'}.items())
            bridge = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(
                    Path(get_package_share_directory('nhk2026_bridge')) /
                    'launch/raspi_can.launch.py')),
                launch_arguments={'non_blocking': LaunchConfiguration('non_blocking')}.items())
            return [
                RegisterEventHandler(OnStateTransition(
                    matcher=lambda event: (
                        isinstance(event, StateTransition) and
                        is_can_bridge(event.action) and event.goal_state == 'active'),
                    entities=[LogInfo(
                        msg='CAN bridge active; starting automatic control'), automatic],
                    handle_once=True)),
                LogInfo(msg='Starting CAN bridge; waiting for its active state'),
                bridge,
            ]

        ack_socket = str(Path(runtime_directory) / 'supervisor.sock')
        joy_local = LaunchConfiguration('joy_source').perform(context) == 'local'
        coordinator = load_restart_coordinator()(
            runtime_actions, ack_socket=ack_socket,
            expected_starts=12 if joy_local else 11,
            supervisor_arguments=[
                '--state-file', LaunchConfiguration('runtime_state_file'),
                '--ack-socket', ack_socket], session_directory=runtime_directory)
        return coordinator.actions()

    return LaunchDescription(arguments + [OpaqueFunction(function=configure_runtime)])

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import StateTransition


def is_can_bridge(action):
    return isinstance(action, LifecycleNode) and action.node_name == '/nhk2026_canbridge'


def stop_on_bridge_exit(event, context):
    if not context.is_shutdown and is_can_bridge(event.action):
        return [EmitEvent(event=Shutdown(reason='CAN bridge exited'))]
    return []


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
            'pump_config', default_value=str(
                workspace / 'src/catchrobo2026_pump/config/pump.yaml')),
        DeclareLaunchArgument('debug', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('listen', default_value='0.0.0.0:8080'),
        DeclareLaunchArgument('ipc_socket', default_value='/tmp/catchrobo2026-control.sock'),
        DeclareLaunchArgument('non_blocking', default_value='true', choices=['true', 'false']),
    ]
    automatic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('catchrobo2026_sequence')) /
            'launch/automatic.launch.py')),
        launch_arguments={name: LaunchConfiguration(name) for name in (
            'team', 'sequence_file', 'queue_config', 'pump_config',
            'debug', 'listen', 'ipc_socket',
        )}.items(),
    )
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('nhk2026_bridge')) /
            'launch/raspi_can.launch.py')),
        launch_arguments={'non_blocking': LaunchConfiguration('non_blocking')}.items(),
    )
    return LaunchDescription(arguments + [
        # Register before launching the bridge so its first activation is observed.
        RegisterEventHandler(OnStateTransition(
            matcher=lambda event: (
                isinstance(event, StateTransition) and
                is_can_bridge(event.action) and event.goal_state == 'active'
            ),
            entities=[LogInfo(msg='CAN bridge active; starting automatic control'), automatic],
            handle_once=True,
        )),
        RegisterEventHandler(OnProcessExit(on_exit=stop_on_bridge_exit)),
        LogInfo(msg='Starting CAN bridge; waiting for its active state'),
        bridge,
    ])

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg
import launch

def generate_launch_description():
    pkg_share = get_package_share_directory('nhk2026_bridge')
    params_file = os.path.join(pkg_share, 'config', 'joy2vel.yml')
    canid_file = os.path.join(pkg_share, 'config', 'test_canbridge.yml')

    name_space = ''
    ld = LaunchDescription()

    # joy2vel node
    joy2velnode = LifecycleNode(
        package='nhk2026_bridge',
        executable='joy_vel_converter',
        name='joy_vel_converter',
        namespace=name_space,
        parameters=[params_file],
        output='screen',
        emulate_tty=True, 
    )

    ld.add_action(joy2velnode)

    joy2vel_converter_configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=joy2velnode,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(joy2velnode),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
        )
    )

    joy2vel_converter_activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=joy2velnode,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(joy2velnode),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    ld.add_action(joy2vel_converter_configure_event_handler)
    ld.add_action(joy2vel_converter_activate_event_handler)

    # test bridge
    test_bridge = LifecycleNode(
        package='nhk2026_bridge',
        executable='test_canbridge',
        name='test_canbridge',
        namespace=name_space,
        parameters=[canid_file],
        output='screen',
        emulate_tty=True, 
    )
    ld.add_action(test_bridge)

    bridge_converter_configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=test_bridge,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(test_bridge),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
        )
    )

    bridge_converter_activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=test_bridge,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(test_bridge),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )
    ld.add_action(bridge_converter_configure_event_handler)
    ld.add_action(bridge_converter_activate_event_handler)

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        namespace=name_space,
        output='screen',
        emulate_tty=True, 
    )
    ld.add_action(joy_node)

    return ld
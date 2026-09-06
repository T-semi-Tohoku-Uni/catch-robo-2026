import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
import lifecycle_msgs.msg


def generate_launch_description():
    package_share = get_package_share_directory('nhk2026_bridge')
    config_file = os.path.join(package_share, 'config', 'raspi_canbridge.yml')
    interface_name = LaunchConfiguration('ifname')

    can_bridge_node = LifecycleNode(
        package='nhk2026_bridge',
        executable='nhk2026_canbridge',
        name='nhk2026_canbridge',
        namespace='',
        parameters=[
            config_file,
            {'ifname': ParameterValue(interface_name, value_type=str)},
        ],
        output='screen',
        emulate_tty=True,
    )

    configure_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=can_bridge_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action == can_bridge_node,
                        transition_id=(
                            lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
                        ),
                    )
                )
            ],
        )
    )

    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=can_bridge_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action == can_bridge_node,
                        transition_id=(
                            lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                        ),
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ifname',
            default_value='vcan0',
            description='Virtual CAN interface used by nhk2026_canbridge',
        ),
        configure_handler,
        activate_handler,
        can_bridge_node,
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='catchrobo2026_endeffector',
            executable='endeffector_state_node',
            name='endeffector_state_node',
            output='screen'
        )
    ])

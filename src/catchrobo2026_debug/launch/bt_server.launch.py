import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("catchrobo2026_debug")
    parameters_file = os.path.join(package_share, "config", "bt_executor.yaml")

    return LaunchDescription(
        [
            Node(
                package="catchrobo2026_debug",
                executable="bt_executor",
                name="bt_action_server",
                output="screen",
                parameters=[parameters_file],
            )
        ]
    )

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav_director_share = get_package_share_directory("nav_director")
    debug_share = get_package_share_directory("catchrobo2026_debug")

    nav_director_launch = os.path.join(nav_director_share, "launch", "test.launch.py")
    bt_server_launch = os.path.join(debug_share, "launch", "bt_server.launch.py")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_director_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bt_server_launch)
            ),
        ]
    )

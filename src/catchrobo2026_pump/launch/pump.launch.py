import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pump_config = os.path.join(
        get_package_share_directory('catchrobo2026_pump'), 'config', 'pump.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'pump_config',
            default_value=pump_config,
            description='Pump controller parameter file'
        ),
        Node(
            package='catchrobo2026_pump',
            executable='pump_controller_node',
            name='pump_controller_node',
            parameters=[LaunchConfiguration('pump_config')],
            output='screen'
        )
    ])

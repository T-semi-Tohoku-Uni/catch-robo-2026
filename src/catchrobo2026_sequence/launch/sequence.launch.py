from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('team', choices=['red', 'blue']),
        DeclareLaunchArgument(
            'sequence_file',
            default_value=PathJoinSubstitution([
                get_package_share_directory('catchrobo2026_sequence'),
                'config', 'sequences.yaml'])),
        DeclareLaunchArgument('debug', default_value='false'),
        Node(
            package='catchrobo2026_sequence', executable='sequence_node',
            output='screen', parameters=[{
                'team': LaunchConfiguration('team'),
                'sequence_file': LaunchConfiguration('sequence_file'),
                'debug': ParameterValue(LaunchConfiguration('debug'), value_type=bool),
            }]),
    ])

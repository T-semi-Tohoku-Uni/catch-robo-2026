import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        DeclareLaunchArgument(
            'use_dummy',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulated joint feedback instead of hardware feedback'
        ),
        DeclareLaunchArgument(
            'initial_pose',
            default_value='[600.0, 200.0, 200.0, 0.0]',
            description='Dummy initial pose [x_mm, y_mm, z_mm, yaw_rad]'
        ),
        # Navigation nodes.
        Node(
            package='nav_director',
            executable='path_generator_3d',
            name='path_generator_3d',
            output='screen'
        ),
        Node(
            package='nav_director',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen'
        ),

        # Manual control.
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='catchrobo2026_hand_operated',
            executable='joy_controller_node',
            name='joy_controller_node',
            output='screen'
        ),

        # Device controllers.
        Node(
            package='catchrobo2026_pump',
            executable='pump_controller_node',
            name='pump_controller_node',
            parameters=[LaunchConfiguration('pump_config')],
            output='screen'
        ),
        Node(
            package='catchrobo2026_endeffector',
            executable='endeffector_state_node',
            name='endeffector_state_node',
            output='screen'
        ),

        # Visualize measured joints.
        Node(
            package='nav_director',
            executable='current_kinematics_visualizer',
            name='current_kinematics_visualizer',
            output='screen'
        ),
        # Simulated feedback is opt-in.
        Node(
            package='nav_director',
            executable='dummy_robot_node',
            name='dummy_robot_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_dummy')),
            parameters=[
                {'initial_pose': LaunchConfiguration('initial_pose')}
            ]
        )
    ])

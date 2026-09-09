from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def package_path(package, *parts):
    return PathJoinSubstitution([get_package_share_directory(package), *parts])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('team', choices=['red', 'blue']),
        DeclareLaunchArgument(
            'sequence_file', default_value=package_path(
                'catchrobo2026_sequence', 'config', 'sequences.yaml')),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument(
            'queue_config', default_value=package_path(
                'catchrobo2026_ui', 'config', 'queue.yaml')),
        DeclareLaunchArgument(
            'pump_config', default_value=package_path(
                'catchrobo2026_pump', 'config', 'pump.yaml')),
        DeclareLaunchArgument(
            'joint_feedback_config', default_value=package_path(
                'nav_director', 'config', 'joint_feedback.yaml')),
        DeclareLaunchArgument('ipc_socket', default_value='/tmp/catchrobo2026-control.sock'),
        DeclareLaunchArgument('listen', default_value='0.0.0.0:8080'),
        
        # Include Launches
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_path(
                'catchrobo2026_sequence', 'launch', 'sequence.launch.py')),
            launch_arguments={
                'team': LaunchConfiguration('team'),
                'sequence_file': LaunchConfiguration('sequence_file'),
                'debug': LaunchConfiguration('debug'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_path(
                'catchrobo2026_ui', 'launch', 'ui.launch.py')),
            launch_arguments={
                'ipc_socket': LaunchConfiguration('ipc_socket'),
                'listen': LaunchConfiguration('listen'),
                'sequence_enabled': 'true',
                'queue_config': LaunchConfiguration('queue_config'),
            }.items()),
            
        # Existing Nodes
        Node(package='nav_director', executable='path_generator_3d',
             parameters=[LaunchConfiguration('joint_feedback_config')], output='screen'),
        Node(package='nav_director', executable='path_follower_node',
             parameters=[LaunchConfiguration('joint_feedback_config')], output='screen'),
        Node(package='catchrobo2026_state_machine', executable='state_machine_node',
             name='state_machine_node', output='screen'),
        Node(package='catchrobo2026_pump', executable='pump_controller_node',
             parameters=[LaunchConfiguration('pump_config')], output='screen'),
        Node(package='catchrobo2026_endeffector', executable='endeffector_state_node',
             output='screen'),

        # ==================================
        # main.launch.py から追加したノード
        # ==================================
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
            parameters=[LaunchConfiguration('joint_feedback_config')],
            output='screen'
        ),
        Node(
            package='nav_director', 
            executable='current_kinematics_visualizer', 
            name='current_kinematics_visualizer', 
            output='screen'
        ),
    ])

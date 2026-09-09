from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'joint_feedback_config', default_value=PathJoinSubstitution([
                get_package_share_directory('nav_director'), 'config', 'joint_feedback.yaml'])),
        # 1. 3D経路生成ノード
        Node(
            package='nav_director',
            executable='path_generator_3d',
            name='path_generator_3d',
            parameters=[LaunchConfiguration('joint_feedback_config')],
            output='screen'
        ),
        
        # 2. 経路追従ノード
        Node(
            package='nav_director',
            executable='path_follower_node',
            name='path_follower_node',
            parameters=[LaunchConfiguration('joint_feedback_config')],
            output='screen'
        ),
        
        # 3. 現在の運動学ビジュアライザノード (ダミーロボットの代わりに追加)
        Node(
            package='nav_director',
            executable='current_kinematics_visualizer',
            name='current_kinematics_visualizer',
            output='screen'
        )
    ])

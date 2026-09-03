from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 3D経路生成ノード
        Node(
            package='nav_director',
            executable='path_generator_3d',
            name='path_generator_3d',
            output='screen'
        ),
        
        # 2. 経路追従ノード
        Node(
            package='nav_director',
            executable='path_follower_node',
            name='path_follower_node',
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
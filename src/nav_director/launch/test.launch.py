from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 3D経路生成ノード
        Node(
            package='nav_director',
            executable='path_generator_3d',
            name='path_generator_3d',
            output='screen' # ターミナルにログを出力
        ),
        
        # 2. 経路追従ノード
        Node(
            package='nav_director',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen'
        ),
        
        # 3. ダミーロボットノード
        Node(
            package='nav_director',
            executable='dummy_robot_node',
            name='dummy_robot_node',
            output='screen',
            parameters=[
                # ここで初期角度を自由に変更できます (例: 1.57は90度)
                {'initial_joints': [600.0, 200.0, 200.0, 0.0]}
            ]
        )
    ])
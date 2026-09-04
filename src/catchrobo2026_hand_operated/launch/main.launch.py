from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ==================================
        # 自動制御 (アクション通信) 系
        # ==================================
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

        # ==================================
        # 手動制御 (Joy) 系
        # ==================================
        # 3. コントローラーノード
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        # 4. Joyからの入力と自動制御からの target_pose を合成するノード
        Node(
            package='catchrobo2026_hand_operated',
            executable='joy_controller_node',
            name='joy_controller_node',
            output='screen'
        ),
        # 5. ポンプ管理ノード
        Node(
            package='catchrobo2026_hand_operated',
            executable='pump_state_node',
            name='pump_state_node',
            output='screen'
        ),

        # ==================================
        # 共通可視化系
        # ==================================
        # 6. 現在の運動学ビジュアライザノード
        Node(
            package='nav_director',
            executable='current_kinematics_visualizer',
            name='current_kinematics_visualizer',
            output='screen'
        )
    ])
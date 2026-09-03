from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. コントローラーのハードウェア入力を読み取るROS 2標準ノード
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        
        # 2. 逆運動学のサービスサーバーノード
        Node(
            package='inverse_kinematics_package',
            executable='inverse_kinematics_node',
            name='inverse_kinematics_node',
            output='screen'
        ),

        # 3. Joy入力を目標座標に変換し、IKを呼び出して関節角度をパブリッシュするノード
        Node(
            package='catchrobo2026_hand_operated',
            executable='joy_controller_node',
            name='joy_controller_node',
            output='screen'
        ),
        
        # 4. 現在のジョイント角度(current_joints)から順運動学を計算し、RViz用のマーカーをパブリッシュするノード
        Node(
            package='nav_director',
            executable='current_kinematics_visualizer',
            name='current_kinematics_visualizer',
            output='screen'
        )
    ])
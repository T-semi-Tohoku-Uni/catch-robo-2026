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
        # 1. コントローラーのハードウェア入力を読み取るROS 2標準ノード
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        
        # 2. Joy入力を目標座標に変換し、IKをローカル計算して関節角度をパブリッシュするノード
        Node(
            package='catchrobo2026_hand_operated',
            executable='joy_controller_node',
            name='joy_controller_node',
            output='screen'
        ),

        # 3. ポンプ・電磁弁制御ノード
        Node(
            package='catchrobo2026_pump',
            executable='pump_controller_node',
            name='pump_controller_node',
            parameters=[LaunchConfiguration('pump_config')],
            output='screen'
        ),
        
        # 4. エンドエフェクタ制御ノード
        Node(
            package='catchrobo2026_endeffector',
            executable='endeffector_state_node',
            name='endeffector_state_node',
            output='screen'
        ),

        # 5. 現在のジョイント角度(current_joints)から順運動学を計算し、RViz用のマーカーをパブリッシュするノード
        Node(
            package='nav_director',
            executable='current_kinematics_visualizer',
            name='current_kinematics_visualizer',
            output='screen'
        )
    ])

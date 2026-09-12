import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pump_config = os.path.join(
        get_package_share_directory('catchrobo2026_pump'), 'config', 'pump.yaml')
    joint_feedback_config = os.path.join(
        get_package_share_directory('nav_director'), 'config', 'joint_feedback.yaml')
    manual_velocity_config = os.path.join(
        get_package_share_directory('catchrobo2026_hand_operated'),
        'config', 'manual_velocity.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'pump_config',
            default_value=pump_config,
            description='Pump controller parameter file'
        ),
        DeclareLaunchArgument(
            'joint_feedback_config', default_value=joint_feedback_config,
            description='Joint feedback tolerance parameter file'
        ),
        DeclareLaunchArgument(
            'manual_velocity_config', default_value=manual_velocity_config,
            description='Manual Cartesian and angular velocity parameter file'
        ),
        DeclareLaunchArgument('debug', default_value='false'),
        # ==================================
        # 自動制御 (アクション通信) 系
        # ==================================
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
            parameters=[LaunchConfiguration('joint_feedback_config'), {
                'goal_joint_tolerance_first_rad': 0.05,  # 配列の1要素目 [rad]
                'goal_joint_tolerance_remaining_rad': 0.05,  # 配列の2〜4要素目 [rad]
            }],
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
            parameters=[LaunchConfiguration('joint_feedback_config'),
                        LaunchConfiguration('manual_velocity_config'), {
                            'debug': ParameterValue(
                                LaunchConfiguration('debug'), value_type=bool),
                            'manual_velocity_config': ParameterValue(
                                LaunchConfiguration('manual_velocity_config'), value_type=str),
                        }],
            output='screen'
        ),
        Node(
            package='catchrobo2026_state_machine',
            executable='state_machine_node',
            name='state_machine_node',
            output='screen'
        ),
        # 5. ポンプ・電磁弁制御ノード
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

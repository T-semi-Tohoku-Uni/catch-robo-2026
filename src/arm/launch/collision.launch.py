from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('robot_arm_description')
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' "', LaunchConfiguration('model'), '"']),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='gakusai_preview',
            description='Namespace for preview topics and TF transport.'),
        GroupAction([
            PushRosNamespace(LaunchConfiguration('namespace')),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            DeclareLaunchArgument(
                'model',
                default_value=PathJoinSubstitution([
                    package_share, 'urdf', 'robot_arm.xacro',
                ]),
                description='Absolute path to the URDF or Xacro model.',
            ),
            DeclareLaunchArgument(
                'fixed_frame', default_value='map',
                description='RViz fixed frame (a link in the model).',
            ),
            DeclareLaunchArgument(
                'publish_joint_states', default_value='true',
                description='Convert four IK angles to URDF joints; disable for external URDF joint states.',
            ),
            DeclareLaunchArgument(
                'initial_angles', default_value='[0.0, 0.0, 0.0, 0.0]',
                description='IK angles [theta1, theta2, theta3, theta4] in radians.',
            ),
            DeclareLaunchArgument('rviz', default_value='true'),
            Node(
                package='robot_arm_description',
                executable='arm_joint_state_publisher.py',
                parameters=[{'initial_angles': LaunchConfiguration('initial_angles')}],
                condition=IfCondition(LaunchConfiguration('publish_joint_states')),
                output='screen',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': robot_description}],
                output='screen',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=[
                    '-d', PathJoinSubstitution([package_share, 'launch', 'collision.rviz']),
                    '-f', LaunchConfiguration('fixed_frame'),
                ],
                output='screen',
            ),
        ]),
    ])

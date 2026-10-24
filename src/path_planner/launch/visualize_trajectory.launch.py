from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_share = FindPackageShare('robot_arm_description')
    planner_share = FindPackageShare('path_planner')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' "', LaunchConfiguration('model'), '"']),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_mode', default_value='demo', choices=['demo', 'yaml']),
        DeclareLaunchArgument('dynamics_config', default_value=''),
        DeclareLaunchArgument('pipeline_config', default_value=''),
        DeclareLaunchArgument(
            'namespace', default_value='gakusai_preview',
            description='Namespace for preview topics and TF transport.'),
        GroupAction([
            PushRosNamespace(LaunchConfiguration('namespace')),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            DeclareLaunchArgument(
                'model',
                default_value=PathJoinSubstitution([arm_share, 'urdf', 'robot_arm.xacro']),
                description='Visualization Xacro/URDF only; planner geometry is unchanged.',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=PathJoinSubstitution(
                    [planner_share, 'rviz', 'trajectory_pipeline.rviz']
                ),
                description='RViz configuration file.',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description='Launch RViz2.',
            ),
            DeclareLaunchArgument(
                'start_angles',
                default_value='[-0.7582, 0.7605, 2.1702, -0.8126]',
                description='Start joint angles [theta1, theta2, theta3, theta4] in rad.',
            ),
            DeclareLaunchArgument(
                'goal_angles',
                default_value='[0.7582, 0.7605, 2.1702, -2.3290]',
                description='Goal joint angles [theta1, theta2, theta3, theta4] in rad.',
            ),
            DeclareLaunchArgument(
                'enable_demo_obstacle',
                default_value=PythonExpression([
                    "'true' if '", LaunchConfiguration('config_mode'),
                    "' == 'demo' else 'false'"]),
                description='Place a spherical obstacle in the middle of the swing path.',
            ),
            DeclareLaunchArgument(
                'obstacle_xyz',
                default_value='[0.675, 0.490, 0.145]',
                description='Obstacle position [x, y, z] in map frame [m].',
            ),
            DeclareLaunchArgument(
                'obstacle_radius',
                default_value='0.065',
                description='Obstacle sphere radius [m].',
            ),
            DeclareLaunchArgument(
                'payload_mass',
                default_value=PythonExpression([
                    "'0.40' if '", LaunchConfiguration('config_mode'),
                    "' == 'demo' else '0.0'"]),
                description='Payload mass at flange [kg].',
            ),
            DeclareLaunchArgument(
                'playback_speed',
                default_value='1.0',
                description='Trajectory animation playback speed multiplier.',
            ),
            DeclareLaunchArgument(
                'seed',
                default_value='42',
                description='Base random seed for RRT-Connect multi-start planning.',
            ),
            Node(
                package='field_visualization',
                executable='map_mesh_publisher',
                name='map_mesh_publisher',
                output='screen',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': robot_description}],
                output='screen',
            ),
            Node(
                package='path_planner',
                executable='trajectory_node',
                name='trajectory_pipeline',
                parameters=[{
                    'config_mode': LaunchConfiguration('config_mode'),
                    'dynamics_config': LaunchConfiguration('dynamics_config'),
                    'pipeline_config': LaunchConfiguration('pipeline_config'),
                    'start_angles': LaunchConfiguration('start_angles'),
                    'goal_angles': LaunchConfiguration('goal_angles'),
                    'enable_demo_obstacle': LaunchConfiguration('enable_demo_obstacle'),
                    'obstacle_xyz': LaunchConfiguration('obstacle_xyz'),
                    'obstacle_radius': LaunchConfiguration('obstacle_radius'),
                    'payload_mass': LaunchConfiguration('payload_mass'),
                    'playback_speed': LaunchConfiguration('playback_speed'),
                    'seed': LaunchConfiguration('seed'),
                }],
                output='screen',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', LaunchConfiguration('rviz_config')],
                output='screen',
            ),
        ]),
    ])

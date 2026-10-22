"""Display only the planned flange path in RViz."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = ParameterValue(
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
            DeclareLaunchArgument('model', default_value=PathJoinSubstitution([
                FindPackageShare('robot_arm_description'), 'urdf', 'robot_arm.xacro'])),
            DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
                FindPackageShare('path_planning'), 'config', 'planner.yaml'])),
            DeclareLaunchArgument('start_planner', default_value='true'),
            DeclareLaunchArgument('rviz', default_value='true'),
            Node(package='path_planning', executable='path_planner_node', output='screen',
                 condition=IfCondition(LaunchConfiguration('start_planner')),
                 parameters=[LaunchConfiguration('params_file'), {'robot_description': description}]),
            Node(package='path_planning', executable='path_preview_node',
                 output='screen', parameters=[{'robot_description': description}]),
            Node(package='rviz2', executable='rviz2', output='screen',
                 condition=IfCondition(LaunchConfiguration('rviz')),
                 arguments=['-d', PathJoinSubstitution([
                     FindPackageShare('path_planning'), 'config', 'preview.rviz'])]),
        ]),
    ])

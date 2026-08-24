from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
import launch_ros
import re

from pathlib import Path
import os

def generate_launch_description():

    core_path = get_package_share_directory('core')
    simulation_path = get_package_share_directory('simulation')

    ws_root = Path(core_path).parents[3]
    field_path = ws_root / 'src' / 'simulation'

    arm_sdf_path = Path(simulation_path) / 'robot_arm' / 'model.sdf'

    models_path_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value=''),
            ':',
            field_path,
        ],
    )

    world_file_path = os.path.join(
        simulation_path, "world", "field.world"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [f' -r 4 {world_file_path}'])]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot_arm',
            '-file', str(arm_sdf_path),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen',
    )

    return LaunchDescription([
        models_path_env,
        gazebo,
        gz_spawn_entity
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # オプション: すでにRViz設定ファイル(.rviz)がある場合はここに追加できます
    rviz_config_dir = os.path.join(get_package_share_directory('field_visualization'), 'rviz', 'field.rviz')

    return LaunchDescription([
        Node(
            package='field_visualization',
            executable='map_mesh_publisher',
            name='map_mesh_publisher',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir], # RViz設定ファイルを使用する場合はコメントアウトを外す
            output='screen'
        )
    ])
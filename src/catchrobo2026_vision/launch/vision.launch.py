"""Independent read-only vision pipeline; no robot control nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {'camera_backend': 'auto', 'camera_device': '/dev/video0',
                'camera_video_path': '', 'camera_video_loop': 'true', 'camera_fps': '20.0',
                'camera_width': '640', 'camera_height': '480', 'vision_port': '8081',
                'vision_bind': '0.0.0.0'}
    def value(name, kind=str):
        return ParameterValue(LaunchConfiguration(name), value_type=kind)
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
        Node(package='camera_publisher', executable='camera_publisher_node', output='screen',
             parameters=[{'backend': value('camera_backend'), 'device': value('camera_device'),
                          'video_path': value('camera_video_path'),
                          'video_loop': value('camera_video_loop', bool),
                          'fps': value('camera_fps', float), 'width': value('camera_width', int),
                          'height': value('camera_height', int)}]),
        Node(package='red_cylinder_detector', executable='red_cylinder_detector_node', output='screen'),
        Node(package='catchrobo2026_vision', executable='vision_http_bridge.py', output='screen',
             parameters=[{'bind': value('vision_bind'), 'port': value('vision_port', int)}]),
    ])

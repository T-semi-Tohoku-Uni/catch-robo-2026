from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    core_path = Path(get_package_share_directory("core"))
    ws_root = core_path.parents[3]
    runner = ws_root / "src" / "simulation" / "script" / "run_genesis_mjcf.py"

    backend = LaunchConfiguration("backend")
    steps = LaunchConfiguration("steps")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_z = LaunchConfiguration("robot_z")
    robot_scale = LaunchConfiguration("robot_scale")

    genesis = ExecuteProcess(
        cmd=[
            "uv",
            "run",
            "python",
            str(runner),
            "--backend",
            backend,
            "--field",
            "--visualization",
            "--viewer",
            "--robot-pos",
            robot_x,
            robot_y,
            robot_z,
            "--robot-scale",
            robot_scale,
            "--steps",
            steps,
            "--hold",
        ],
        cwd=str(ws_root),
        additional_env={
            "MPLCONFIGDIR": "/tmp/matplotlib",
        },
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="gpu"),
            DeclareLaunchArgument("steps", default_value="0"),
            DeclareLaunchArgument("robot_x", default_value="0.0"),
            DeclareLaunchArgument("robot_y", default_value="0.0"),
            DeclareLaunchArgument("robot_z", default_value="0.27"),
            DeclareLaunchArgument("robot_scale", default_value="10.0"),
            genesis,
        ]
    )

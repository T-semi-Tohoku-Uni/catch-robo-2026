from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import re
import subprocess

from launch.actions import EmitEvent, RegisterEventHandler
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg
import launch

def _require_can0(context, *args, **kwargs):
    # Check interface availability.
    try:
        subprocess.check_output(["/usr/sbin/ip", "link", "show", "can0"], stderr=subprocess.STDOUT, text=True)
        return []
    except subprocess.CalledProcessError as e:
        # Stop if the interface is unavailable.
        msg = f"can0 が見つかりません。launch を停止します。\n(ip output: {e.output.strip()})"
        launch.logging.get_logger(__name__).error(msg)
        raise RuntimeError("can0 not found") from e
    except FileNotFoundError:
        # Stop if ip is unavailable.
        msg = "ip コマンドが見つかりません。launch を停止します。"
        launch.logging.get_logger(__name__).error(msg)
        raise RuntimeError("ip command not found")

def _run_cmd(cmd):
    try:
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
        return True, out
    except subprocess.CalledProcessError as e:
        return False, e.output

def _ensure_can0_up(context, *args, **kwargs):
    # Read the current interface configuration.
    try:
        out = subprocess.check_output(["/usr/sbin/ip", "-details", "link", "show", "can0"], text=True)
    except Exception as error:
        raise RuntimeError("Cannot read can0 configuration") from error

    # ip reports CAN FD as a controller flag, not as "fd on".
    link_flags = re.search(r"^\d+:\s+can0(?:@\S+)?:\s+<([^>]*)>", out, re.MULTILINE)
    can_flags = re.search(r"^\s+can\s+<([^>]*)>", out, re.MULTILINE)
    desired_ok = (
        can_flags is not None
        and "FD" in can_flags.group(1).split(",")
        and re.search(r"^\s+bitrate\s+1000000(?:\s|$)", out, re.MULTILINE)
        and re.search(r"^\s+dbitrate\s+2000000(?:\s|$)", out, re.MULTILINE)
    )
    if link_flags and "UP" in link_flags.group(1).split(",") and desired_ok:
        return []

    # Configure a down or mismatched interface with the existing commands.
    cmds = [
        ["sudo", "-n", "/usr/sbin/ip", "link", "set", "can0", "down"],
        [
            "sudo", "-n", "/usr/sbin/ip", "link", "set", "can0", "up",
            "type", "can",
            "bitrate", "1000000",
            "dbitrate", "2000000",
            "fd", "on",
        ],
    ]
    for cmd in cmds:
        ok, out = _run_cmd(cmd)
        if not ok:
            launch.logging.get_logger(__name__).error(
                "can0 設定に失敗しました。cmd: %s\n(output: %s)"
                % (" ".join(cmd), out.strip())
            )
            raise RuntimeError("can0 setup failed")
    return []

def _initialization_failed(context):
    raise RuntimeError("CAN-RPC initialization failed; automatic control was not started")


def generate_launch_description():
    pkg_share = get_package_share_directory('canrpc_bridge')
    canid_file = os.path.join(pkg_share, 'config', 'raspi_canbridge.yml')
    non_blocking = LaunchConfiguration('non_blocking')
    reset_on_startup = LaunchConfiguration('reset_on_startup')
    startup_timeout_sec = LaunchConfiguration('startup_timeout_sec')

    name_space = ''
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            'non_blocking',
            default_value='true',
            choices=['true', 'false'],
            description='Use non-blocking SocketCAN transmission',
        )
    )
    ld.add_action(DeclareLaunchArgument('reset_on_startup',default_value='false',choices=['true','false']))
    ld.add_action(DeclareLaunchArgument('startup_timeout_sec',default_value='30.0'))
    ld.add_action(OpaqueFunction(function=_require_can0))
    ld.add_action(OpaqueFunction(function=_ensure_can0_up))

    # Create the lifecycle bridge.
    canbridgenode = LifecycleNode(
        package='canrpc_bridge',
        executable='canrpc_bridge',
        name='canrpc_bridge',
        namespace=name_space,
        parameters=[
            canid_file,
            {'non_blocking': ParameterValue(non_blocking, value_type=bool),
             'reset_on_startup': ParameterValue(reset_on_startup, value_type=bool),
             'startup_timeout_sec': ParameterValue(startup_timeout_sec, value_type=float)},
        ],
        output='screen',
        emulate_tty=True,
    )

    ld.add_action(canbridgenode)

    canbridge_configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=canbridgenode,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(canbridgenode),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
        )
    )

    canbridge_activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=canbridgenode,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(canbridgenode),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    ld.add_action(canbridge_configure_event_handler)
    ld.add_action(canbridge_activate_event_handler)

    for start, goal in [('activating','inactive'), ('configuring','unconfigured')]:
        ld.add_action(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=canbridgenode, start_state=start, goal_state=goal,
            entities=[OpaqueFunction(function=_initialization_failed)],
        )))
    ld.add_action(RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=canbridgenode, goal_state='errorprocessing',
        entities=[OpaqueFunction(function=_initialization_failed)],
    )))
    return ld

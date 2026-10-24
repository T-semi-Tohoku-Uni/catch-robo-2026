"""Load a portable project profile before configuring a SocketCAN interface."""
from pathlib import Path
import importlib.util
import re
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def load_profile(path):
    validator = Path(__file__).resolve().parent.parent / 'tools' / 'canrpc_profile.py'
    spec = importlib.util.spec_from_file_location('canrpc_profile', validator)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.validate_profile(path)


def configure_interface(bus):
    if not bus['configure']:
        return
    interface = bus['interface']
    ip = shutil.which('ip')
    if not ip:
        raise RuntimeError('ip command not found')
    try:
        output = subprocess.check_output([ip, '-details', 'link', 'show', interface],
                                         stderr=subprocess.STDOUT, text=True)
    except (OSError, subprocess.CalledProcessError) as error:
        raise RuntimeError(f'Cannot read CAN interface {interface}') from error
    link = re.search(r'^\d+:\s+' + re.escape(interface) + r'(?:@\S+)?:\s+<([^>]*)>', output, re.MULTILINE)
    flags = re.search(r'^\s+can\s+<([^>]*)>', output, re.MULTILINE)
    nominal = str(bus['nominal_bitrate'])
    data = str(bus['data_bitrate'])
    ready = (link and 'UP' in link.group(1).split(',') and flags and
             'FD' in flags.group(1).split(',') and
             re.search(r'^\s+bitrate\s+' + nominal + r'(?:\s|$)', output, re.MULTILINE) and
             re.search(r'^\s+dbitrate\s+' + data + r'(?:\s|$)', output, re.MULTILINE))
    if ready:
        return
    commands = [
        ['sudo', '-n', ip, 'link', 'set', interface, 'down'],
        ['sudo', '-n', ip, 'link', 'set', interface, 'up', 'type', 'can',
         'bitrate', nominal, 'dbitrate', data, 'fd', 'on'],
    ]
    for command in commands:
        try:
            subprocess.check_output(command, stderr=subprocess.STDOUT, text=True)
        except (OSError, subprocess.CalledProcessError) as error:
            raise RuntimeError(f'CAN interface setup failed: {interface}') from error


def initialization_failed(context):
    return [LogInfo(msg='ERROR: CAN-RPC lifecycle transition failed; diagnostic processes remain alive')]


def start_bridge(context):
    path = LaunchConfiguration('config_file').perform(context)
    profile = load_profile(path)
    reset = LaunchConfiguration('reset_on_startup').perform(context)
    timeout = float(LaunchConfiguration('startup_timeout_sec').perform(context))
    if reset not in ('true', 'false'):
        raise RuntimeError('reset_on_startup must be true or false')
    if timeout < 0 or not timeout < 2147483647 / 1000:
        raise RuntimeError('invalid startup_timeout_sec override')
    if reset == 'true' and not profile['reset']['targets']:
        raise RuntimeError('reset_on_startup requires reset.targets')
    # Validate the complete profile before touching the interface.
    setup_error = ''
    try:
        configure_interface(profile['bus'])
    except RuntimeError as error:
        setup_error = str(error)
    node = LifecycleNode(package='canrpc_bridge', executable='canrpc_bridge',
                         name='canrpc_bridge', namespace='', output='screen', emulate_tty=True,
                         parameters=[{'config_file': path, 'reset_on_startup': reset == 'true',
                                      'startup_timeout_sec': timeout, 'startup_block_reason': setup_error,
                                      'status_topic': LaunchConfiguration('status_topic').perform(context)}])
    handlers = [
        RegisterEventHandler(OnProcessStart(target_action=node, on_start=[EmitEvent(
            event=ChangeState(lifecycle_node_matcher=matches_action(node),
                              transition_id=Transition.TRANSITION_CONFIGURE))])),
    ]
    if not setup_error:
        handlers.append(RegisterEventHandler(OnStateTransition(target_lifecycle_node=node,
            start_state='configuring', goal_state='inactive', entities=[EmitEvent(
                event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                  transition_id=Transition.TRANSITION_ACTIVATE))], handle_once=True)))
    else:
        handlers.append(LogInfo(msg='ERROR: ' + setup_error + '; activation withheld. Prepare the interface and explicitly activate.'))

    for start, goal in [('activating', 'inactive'), ('configuring', 'unconfigured')]:
        handlers.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node, start_state=start, goal_state=goal,
            entities=[OpaqueFunction(function=initialization_failed)])))
    handlers.append(RegisterEventHandler(OnStateTransition(target_lifecycle_node=node,
        goal_state='errorprocessing', entities=[OpaqueFunction(function=initialization_failed)])))
    return [*handlers, node]


def generate_launch_description():
    profile = str(Path(get_package_share_directory('canrpc_bridge')) / 'config' / 'gakusai.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=profile),
        DeclareLaunchArgument('status_topic', default_value='canrpc/status'),
        DeclareLaunchArgument('reset_on_startup', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('startup_timeout_sec', default_value='0.0',
                              description='0 uses the profile timeout; positive values override it'),
        OpaqueFunction(function=start_bridge),
    ])

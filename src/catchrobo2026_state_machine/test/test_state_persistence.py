"""Verify initialization state persistence across process recreation."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from catchrobo2026_msgs.srv import StateControl
import rclpy
from std_msgs.msg import Int32MultiArray


def start(executable, namespace, state_file, log):
    return subprocess.Popen(
        [str(executable), '--ros-args', '-r', f'__ns:={namespace}',
         '-p', f'state_file:={state_file}', '-p', 'require_state_file:=true'],
        stdout=log, stderr=log, start_new_session=True)


def stop(child):
    if child.poll() is None:
        os.killpg(child.pid, signal.SIGINT)
        child.wait(timeout=5.0)


def spin_until(node, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline
        rclpy.spin_once(node, timeout_sec=0.02)


def test_state_is_restored_before_first_publish():
    os.environ['ROS_DOMAIN_ID'] = '231'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    rclpy.init()
    node = rclpy.create_node('state_persistence_test', namespace=f'/state_{os.getpid()}')
    values = []
    observer = node.create_subscription(
        Int32MultiArray, 'init_state', lambda msg: values.append(msg.data[0]), 10)
    client = node.create_client(StateControl, 'set_value')
    executable = (Path(get_package_prefix('catchrobo2026_state_machine')) /
                  'lib/catchrobo2026_state_machine/state_machine_node')
    log = tempfile.TemporaryFile(mode='w+')
    with tempfile.TemporaryDirectory() as directory:
        session = Path(directory) / 'session'
        session.mkdir()
        state_file = session / 'init-state'
        state_file.write_text('0\n', encoding='ascii')
        child = start(executable, node.get_namespace(), state_file, log)
        try:
            spin_until(node, lambda: client.service_is_ready() and bool(values))
            assert values[-1] == 0
            future = client.call_async(StateControl.Request(command=7))
            spin_until(node, future.done)
            assert future.result().success
            assert state_file.read_text(encoding='ascii') == '7\n'
            spin_until(node, lambda: values[-1] == 7)
            stop(child)
            node.destroy_subscription(observer)
            values.clear()
            observer = node.create_subscription(
                Int32MultiArray, 'init_state',
                lambda msg: values.append(msg.data[0]), 10)
            child = start(executable, node.get_namespace(), state_file, log)
            spin_until(node, lambda: bool(values))
            assert values[0] == 7

            moved_session = Path(directory) / 'moved-session'
            session.rename(moved_session)
            future = client.call_async(StateControl.Request(command=8))
            spin_until(node, future.done)
            assert not future.result().success
            values.clear()
            spin_until(node, lambda: bool(values))
            assert set(values) == {7}
        finally:
            stop(child)
    log.close()
    node.destroy_node()
    rclpy.shutdown()


def test_required_state_file_rejects_missing_and_corrupt_input():
    executable = (Path(get_package_prefix('catchrobo2026_state_machine')) /
                  'lib/catchrobo2026_state_machine/state_machine_node')
    log = tempfile.TemporaryFile(mode='w+')
    with tempfile.TemporaryDirectory() as directory:
        state_file = Path(directory) / 'init-state'
        for contents in (None, 'broken\n'):
            if contents is None:
                state_file.unlink(missing_ok=True)
            else:
                state_file.write_text(contents, encoding='ascii')
            child = start(executable, f'/invalid_state_{os.getpid()}', state_file, log)
            assert child.wait(timeout=5.0) != 0
    log.close()

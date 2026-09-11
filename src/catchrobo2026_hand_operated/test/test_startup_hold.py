"""Verify restart-safe Joy startup without CAN hardware."""

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from std_msgs.msg import Float32MultiArray


def spin_until(node, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline
        rclpy.spin_once(node, timeout_sec=0.02)


def test_restart_safe_startup_holds_first_valid_feedback():
    os.environ['ROS_DOMAIN_ID'] = '230'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    rclpy.init()
    node = rclpy.create_node('startup_hold_test', namespace=f'/startup_{os.getpid()}')
    samples = []
    node.create_subscription(
        Float32MultiArray, 'target_joint_angles',
        lambda message: samples.append(list(message.data)), 100)
    feedback = node.create_publisher(Float32MultiArray, 'current_joints', 10)
    log = tempfile.TemporaryFile(mode='w+')
    executable = (Path(get_package_prefix('catchrobo2026_hand_operated')) /
                  'lib/catchrobo2026_hand_operated/joy_controller_node')
    child = subprocess.Popen(
        [str(executable), '--ros-args', '-r', f'__ns:={node.get_namespace()}',
         '-p', 'require_current_joints_on_start:=true'],
        stdout=log, stderr=log, start_new_session=True)
    try:
        spin_until(node, lambda: feedback.get_subscription_count() > 0)
        end = time.monotonic() + 0.2
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert samples == []

        invalid = Float32MultiArray(data=[0.1, 0.2, 0.3, math.radians(7.0)])
        feedback.publish(invalid)
        end = time.monotonic() + 0.2
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert samples == []

        measured = [0.1, 0.2, 0.3, math.radians(5.0)]
        feedback.publish(Float32MultiArray(data=measured))
        spin_until(node, lambda: bool(samples))
        expected = measured[:3] + [0.0]
        assert all(abs(actual - wanted) < 1e-6
                   for actual, wanted in zip(samples[-1], expected))
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            child.wait(timeout=5.0)
        log.close()
        node.destroy_node()
        rclpy.shutdown()

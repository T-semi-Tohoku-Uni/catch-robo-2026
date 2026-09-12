"""Verify manual velocity YAML reload behavior with an isolated Joy node."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray

os.environ['ROS_DOMAIN_ID'] = '229'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_LOCALHOST_ONLY', None)


class ManualVelocityReloadTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.executable = (Path(get_package_prefix('catchrobo2026_hand_operated')) /
                          'lib/catchrobo2026_hand_operated/joy_controller_node')
        cls.counter = 0

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        type(self).counter += 1
        self.namespace = f'/manual_reload_{os.getpid()}_{self.counter}'
        self.temp_dir = tempfile.TemporaryDirectory()
        self.config = Path(self.temp_dir.name) / 'manual_velocity.yaml'
        self.child = None
        self.node = None
        self.log = None
        self.samples = []
        self.joint_samples = []

    def tearDown(self):
        if self.child is not None and self.child.poll() is None:
            os.killpg(self.child.pid, signal.SIGINT)
            try:
                self.child.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.child.pid, signal.SIGKILL)
                self.child.wait(timeout=3.0)
        if self.log is not None:
            self.log.seek(0)
            print(self.log.read())
            self.log.close()
        if self.node is not None:
            self.node.destroy_node()
        self.temp_dir.cleanup()

    def write_config(self, linear=None, angular=None, node='/**/joy_controller_node'):
        lines = [f'{node}:', '  ros__parameters:']
        if linear is not None:
            lines.append(f'    manual_linear_speed_mm_s: {linear}')
        if angular is not None:
            lines.append(f'    manual_angular_speed_rad_s: {angular}')
        self.config.write_text('\n'.join(lines) + '\n', encoding='utf-8')

    def start_node(self, debug=True, linear=20.0, angular=0.2):
        self.write_config(linear, angular)
        self.node = rclpy.create_node('reload_test', namespace=self.namespace)
        self.node.create_subscription(
            MarkerArray, 'target_arm_markers',
            lambda message: self.samples.append(
                message.markers[-1].points[-1].x), 100)
        self.node.create_subscription(
            Float32MultiArray, 'target_joint_angles',
            lambda message: self.joint_samples.append(
                message.data[0] + message.data[3]), 100)
        self.joy_pub = self.node.create_publisher(Joy, 'joy', 10)
        self.log = tempfile.TemporaryFile(mode='w+')
        self.child = subprocess.Popen(
            [str(self.executable), '--ros-args', '-r', f'__ns:={self.namespace}',
             '--params-file', str(self.config),
             '-p', f'debug:={str(debug).lower()}',
             '-p', f'manual_velocity_config:={self.config}'],
            stdout=self.log, stderr=self.log, start_new_session=True)
        deadline = time.monotonic() + 10.0
        while not (self.samples and self.joy_pub.get_subscription_count()):
            if time.monotonic() >= deadline or self.child.poll() is not None:
                self.fail('Joy node did not become ready')
            rclpy.spin_once(self.node, timeout_sec=0.02)

    @staticmethod
    def joy(x=0.0, wrist=0.0, button=False):
        message = Joy()
        message.axes = [x, 0.0, 1.0, wrist, 0.0, 1.0, 0.0, 0.0]
        message.buttons = [0, 0, 0, 0, 0, int(button)]
        return message

    def spin(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.assertIsNone(self.child.poll())
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def publish_and_spin(self, message, duration=0.08):
        self.samples.clear()
        self.joy_pub.publish(message)
        self.spin(duration)
        return list(self.samples)

    def assert_linear_speed(self, expected_mm_s):
        samples = self.publish_and_spin(self.joy(x=1.0))
        self.assertGreater(len(samples), 5)
        expected_step_m = -expected_mm_s * 0.002 / 1000.0
        for first, second in zip(samples[-6:-1], samples[-5:]):
            self.assertAlmostEqual(second - first, expected_step_m, delta=2e-6)

    def assert_angular_speed(self, expected_rad_s):
        self.joint_samples.clear()
        self.joy_pub.publish(self.joy(wrist=1.0))
        self.spin(0.08)
        self.assertGreater(len(self.joint_samples), 5)
        expected_step_rad = expected_rad_s * 0.002
        for first, second in zip(self.joint_samples[-6:-1], self.joint_samples[-5:]):
            self.assertAlmostEqual(second - first, expected_step_rad, delta=2e-5)

    def assert_neutral_motion(self, duration=0.10):
        samples = self.publish_and_spin(self.joy(x=1.0), duration)
        self.assert_samples_are_stationary(samples)

    def assert_samples_are_stationary(self, samples):
        self.assertGreater(len(samples), 5)
        for first, second in zip(samples[-6:-1], samples[-5:]):
            self.assertAlmostEqual(second, first, delta=1e-8)

    def neutral(self, duration=0.06):
        self.publish_and_spin(self.joy(), duration)

    def test_debug_reload_is_atomic_and_waits_for_neutral_boundary(self):
        self.start_node(debug=True)
        self.assert_linear_speed(20.0)

        self.write_config(40.0, 0.4)
        self.spin(0.65)
        self.samples.clear()
        self.spin(0.08)
        active_samples = list(self.samples)
        self.assertGreater(len(active_samples), 5)
        for first, second in zip(active_samples[-6:-1], active_samples[-5:]):
            self.assertAlmostEqual(second - first, -0.00004, delta=2e-6)

        self.neutral(0.65)
        self.assert_linear_speed(40.0)
        self.neutral()
        self.assert_angular_speed(0.4)

        self.neutral()
        self.write_config(None, 0.6)
        self.spin(0.65)
        self.assert_linear_speed(50.0)
        self.neutral()
        self.assert_angular_speed(0.6)

    def test_invalid_or_missing_config_blocks_until_real_neutral_and_recovers(self):
        self.start_node(debug=True)
        self.neutral()

        self.write_config(-1.0, 0.2)
        self.spin(0.65)
        self.assert_neutral_motion()

        self.write_config(30.0, 0.3)
        self.spin(0.65)
        self.assert_neutral_motion()
        self.neutral(0.65)
        self.assert_linear_speed(30.0)

        self.neutral()
        self.config.unlink()
        self.spin(0.65)
        self.assert_neutral_motion()
        self.neutral()
        self.write_config(35.0, 0.35, node='/**/wrong_node')
        self.spin(0.65)
        self.assert_neutral_motion()
        self.neutral()
        self.write_config(35.0, 0.35)
        with self.config.open('a', encoding='utf-8') as stream:
            stream.write('    manual_linear_speed_mm_ss: 99.0\n')
        self.spin(0.65)
        self.assert_neutral_motion()
        self.neutral()
        self.write_config('3.5e38', 0.35)
        self.spin(0.65)
        self.assert_neutral_motion()
        self.neutral()
        self.write_config(15.0, 0.15)
        self.spin(0.65)
        self.assert_linear_speed(15.0)

    def test_normal_mode_keeps_startup_values(self):
        self.start_node(debug=False, linear=12.0, angular=0.12)
        self.neutral()
        self.write_config(60.0, 0.6)
        self.spin(0.65)
        self.assert_linear_speed(12.0)

    def test_buttons_and_only_command_axes_define_neutral(self):
        self.start_node(debug=True)
        self.neutral()
        self.write_config(-1.0, 0.2)
        self.spin(0.65)

        self.publish_and_spin(self.joy(button=True))
        self.write_config(22.0, 0.22)
        self.spin(0.65)
        self.assert_neutral_motion()
        self.neutral(0.65)
        self.assert_linear_speed(22.0)

        malformed = Joy()
        malformed.axes = [1.0]
        malformed.buttons = [0]
        self.assert_samples_are_stationary(self.publish_and_spin(malformed))
        self.assert_neutral_motion()
        self.neutral()

        nonfinite = self.joy(x=float('nan'))
        self.assert_samples_are_stationary(self.publish_and_spin(nonfinite))
        self.assert_neutral_motion()
        self.neutral()
        self.assert_linear_speed(22.0)


if __name__ == '__main__':
    unittest.main()

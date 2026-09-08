"""Exercise the real Joy wrist service using isolated ROS topics, without CAN."""

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
from catchrobo2026_msgs.srv import WristControl
from geometry_msgs.msg import PoseStamped
import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

os.environ['ROS_DOMAIN_ID'] = '232'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_LOCALHOST_ONLY', None)


class WristControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('wrist_test', namespace=f'/wrist_test_{os.getpid()}')
        cls.samples = []
        cls.initializations = []
        cls.node.create_subscription(
            Float32MultiArray, 'target_joint_angles',
            lambda message: cls.samples.append((time.monotonic(), list(message.data))), 100)
        cls.pose_pub = cls.node.create_publisher(PoseStamped, 'target_pose', 10)
        cls.joints_pub = cls.node.create_publisher(Float32MultiArray, 'current_joints', 10)
        cls.joy_pub = cls.node.create_publisher(Joy, 'joy', 10)
        cls.client = cls.node.create_client(WristControl, 'wrist_control')

        def initialize(request, response):
            cls.initializations.append(request)
            response.success = True
            return response

        cls.node.create_service(Trigger, 'request_initialization', initialize)
        cls.log = tempfile.TemporaryFile(mode='w+')
        executable = (Path(get_package_prefix('catchrobo2026_hand_operated')) /
                      'lib/catchrobo2026_hand_operated/joy_controller_node')
        cls.child = subprocess.Popen(
            [str(executable), '--ros-args', '-r', f'__ns:={cls.node.get_namespace()}',
             '-p', 'rotation_joint_timeout_sec:=0.3'],
            stdout=cls.log, stderr=cls.log, start_new_session=True)
        deadline = time.monotonic() + 10.0
        while not (cls.client.service_is_ready() and cls.samples and
                   cls.pose_pub.get_subscription_count() and
                   cls.joints_pub.get_subscription_count() and
                   cls.joy_pub.get_subscription_count()):
            if time.monotonic() >= deadline or cls.child.poll() is not None:
                cls.tearDownClass()
                raise RuntimeError('Joy service did not become ready')
            rclpy.spin_once(cls.node, timeout_sec=0.02)
        cls.group_counter = 1000

    @classmethod
    def tearDownClass(cls):
        if cls.child.poll() is None:
            os.killpg(cls.child.pid, signal.SIGINT)
            try:
                cls.child.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(cls.child.pid, signal.SIGKILL)
                cls.child.wait(timeout=3.0)
        cls.log.seek(0)
        print(cls.log.read())
        cls.log.close()
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        type(self).group_counter += 1
        self.group_id = self.group_counter
        self.active = False
        self.joy_pub.publish(self.joy())
        self.spin(0.04)

    def tearDown(self):
        if self.active:
            self.request(WristControl.Request.END)
        self.joy_pub.publish(self.joy())
        self.spin(0.04)

    def spin(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.assertIsNone(self.child.poll())
            rclpy.spin_once(self.node, timeout_sec=0.01)

    @staticmethod
    def pose(wrist=-2.0, x=0.675):
        result = PoseStamped()
        result.header.frame_id = 'map'
        result.pose.position.x = x
        result.pose.position.y = 0.2
        result.pose.position.z = 0.2
        yaw = math.atan2(x - 0.675, 0.39) + wrist
        cp, sp = math.cos(-math.pi / 4.0), math.sin(-math.pi / 4.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        result.pose.orientation.x = -sp * sy
        result.pose.orientation.y = sp * cy
        result.pose.orientation.z = cp * sy
        result.pose.orientation.w = cp * cy
        return result

    @staticmethod
    def joints(wrist=-2.0):
        # A known reachable posture at [675, 200, 200] mm.
        elbow = math.acos((390.0**2 + 110.0**2 - 2 * 480.0**2) / (2 * 480.0**2))
        shoulder = math.atan2(390.0, 110.0) - elbow / 2.0
        return [0.0, shoulder, shoulder + elbow, wrist]

    @staticmethod
    def joy(x=0.0, wrist=0.0, initialize=False):
        result = Joy()
        result.axes = [x, 0.0, 0.0, wrist, 0.0, 0.0]
        result.buttons = [int(initialize), 0, 0, 0, 0, 0]
        return result

    def feedback(self, wrist=-2.0, values=None):
        result = Float32MultiArray()
        result.data = self.joints(wrist) if values is None else values
        self.joints_pub.publish(result)
        self.spin(0.04)

    def request(self, operation, wrist=-2.0, direction=1, pose=None, group_id=None):
        request = WristControl.Request()
        request.operation = operation
        request.group_id = self.group_id if group_id is None else group_id
        request.direction = direction
        request.wrist_angle = wrist
        request.target = self.pose(wrist) if pose is None else pose
        future = self.client.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done():
            self.assertLess(time.monotonic(), deadline, 'Wrist service timed out')
            rclpy.spin_once(self.node, timeout_sec=0.01)
        response = future.result()
        if response.success and request.group_id == self.group_id:
            if operation == WristControl.Request.BEGIN:
                self.active = True
            elif operation == WristControl.Request.END:
                self.active = False
        return response

    def begin(self, wrist=-2.0, direction=1):
        self.feedback(wrist)
        result = self.request(WristControl.Request.BEGIN, wrist, direction)
        self.assertTrue(result.success, result.message)

    def assert_hold(self, expected, duration=0.16):
        start = time.monotonic()
        self.spin(duration)
        settled = [sample for stamp, sample in self.samples if stamp > start + 0.04]
        self.assertGreaterEqual(len(settled), 3)
        for sample in settled:
            self.assertEqual(len(sample), 4)
            for actual, target in zip(sample, expected):
                self.assertTrue(math.isfinite(actual))
                self.assertAlmostEqual(actual, target, delta=2e-6)

    def test_00_missing_invalid_stale_feedback_and_start_mismatch(self):
        self.assertFalse(self.request(WristControl.Request.BEGIN).success)
        for values in ([], [0.0, 1.0], [0.0, math.nan, 1.0, -2.0], self.joints(0.1)):
            self.feedback(values=values)
            self.assertFalse(self.request(WristControl.Request.BEGIN).success)
        self.feedback()
        self.spin(0.35)
        self.assertFalse(self.request(WristControl.Request.BEGIN).success)
        self.feedback()
        self.assertFalse(self.request(WristControl.Request.BEGIN, wrist=-1.0).success)
        self.assertFalse(self.request(
            WristControl.Request.BEGIN, pose=self.pose(x=0.775)).success)
        self.assertFalse(self.request(
            WristControl.Request.BEGIN, pose=self.pose(wrist=-1.0)).success)
        self.assertFalse(self.request(WristControl.Request.BEGIN, direction=0).success)
        self.assertFalse(self.request(WristControl.Request.BEGIN, group_id=0).success)
        self.begin()

    def test_begin_holds_feedback_and_suppresses_legacy_joy_and_initialization(self):
        self.pose_pub.publish(self.pose(wrist=-4.0, x=0.8))
        self.joy_pub.publish(self.joy(x=1.0, wrist=1.0))
        self.spin(0.06)
        self.begin()
        self.assert_hold(self.joints())
        initializations = len(self.initializations)
        self.pose_pub.publish(self.pose(wrist=-4.0))
        self.joy_pub.publish(self.joy(x=1.0, wrist=1.0, initialize=True))
        self.assert_hold(self.joints())
        self.assertEqual(len(self.initializations), initializations)
        self.assertFalse(self.request(
            WristControl.Request.BEGIN, group_id=self.group_id + 1).success)

    def test_monotonic_target_then_reverse_fault_and_end_hold(self):
        self.begin()
        self.assertTrue(self.request(WristControl.Request.TARGET, wrist=-1.0).success)
        self.assert_hold(self.joints(-1.0))
        self.assertFalse(self.request(WristControl.Request.TARGET, wrist=-1.1).success)
        self.assertFalse(self.request(WristControl.Request.TARGET, wrist=-0.9).success)
        self.pose_pub.publish(self.pose(wrist=-4.0))
        self.assert_hold(self.joints(-1.0))
        self.assertTrue(self.request(WristControl.Request.END).success)
        self.assert_hold(self.joints(-1.0))
        self.assertTrue(self.request(WristControl.Request.END).success)
        self.assertFalse(self.request(WristControl.Request.TARGET, wrist=-0.9).success)
        self.assertFalse(self.request(WristControl.Request.BEGIN).success)
        self.assert_hold(self.joints(-1.0))

    def test_negative_direction_and_end_endpoint_without_rewrap(self):
        self.begin(wrist=-5.0, direction=-1)
        self.assertTrue(self.request(WristControl.Request.TARGET, wrist=-2.0 * math.pi).success)
        self.assert_hold(self.joints(-2.0 * math.pi))
        self.assertFalse(self.request(WristControl.Request.TARGET, wrist=-6.1).success)
        self.assertTrue(self.request(WristControl.Request.END).success)
        self.joy_pub.publish(self.joy())
        self.assert_hold(self.joints(-2.0 * math.pi))

    def test_quaternion_yaw_wrap_and_small_roundoff_keep_wrist_monotonic(self):
        self.begin(wrist=-4.0)
        for wrist in (-3.5, -3.0, -2.5):
            self.assertTrue(self.request(WristControl.Request.TARGET, wrist=wrist).success)
            self.assert_hold(self.joints(wrist))
        self.assertTrue(self.request(WristControl.Request.TARGET, wrist=-2.500005).success)
        self.assert_hold(self.joints(-2.5))

    def test_target_validation_faults_hold_all_four_joints(self):
        invalid_targets = []
        invalid_targets.append((0.2, self.pose(0.2)))
        invalid_targets.append((-1.0, self.pose(-1.5)))
        unreachable = self.pose(-1.0)
        unreachable.pose.position.z = 10.0
        invalid_targets.append((-1.0, unreachable))
        invalid_pose = self.pose(-1.0)
        invalid_pose.pose.orientation.w = math.nan
        invalid_targets.append((-1.0, invalid_pose))
        invalid_targets.append((math.nan, self.pose(-1.0)))
        for wrist, pose in invalid_targets:
            with self.subTest(wrist=wrist, pose=pose):
                self.begin()
                self.assertFalse(self.request(
                    WristControl.Request.TARGET, wrist=wrist, pose=pose).success)
                self.assertFalse(self.request(WristControl.Request.TARGET, wrist=-1.0).success)
                self.assert_hold(self.joints())
                self.assertTrue(self.request(WristControl.Request.END).success)
                type(self).group_counter += 1
                self.group_id = self.group_counter

    def test_wrong_group_id_does_not_fault_or_end_current_group(self):
        self.begin()
        self.assertFalse(self.request(WristControl.Request.TARGET, wrist=-1.0,
                                      group_id=self.group_id - 1).success)
        self.assertFalse(self.request(
            WristControl.Request.END, group_id=self.group_id - 1).success)
        self.assertTrue(self.request(WristControl.Request.TARGET, wrist=-1.0).success)
        self.assert_hold(self.joints(-1.0))

    def test_post_end_new_pose_and_new_joy_resume_normal_control(self):
        self.begin()
        self.assertTrue(self.request(WristControl.Request.END).success)
        self.assert_hold(self.joints())
        self.pose_pub.publish(self.pose(-3.0))
        self.assert_hold(self.joints(-3.0))
        type(self).group_counter += 1
        self.group_id = self.group_counter
        self.begin()
        self.joy_pub.publish(self.joy(wrist=1.0))
        self.assert_hold(self.joints())
        self.assertTrue(self.request(WristControl.Request.END).success)
        self.assert_hold(self.joints())
        self.joy_pub.publish(self.joy(wrist=1.0))
        self.spin(0.1)
        self.assertGreater(self.samples[-1][1][3], -1.9)
        last_wrist = self.samples[-1][1][3]
        self.assertTrue(self.request(WristControl.Request.END).success)
        self.spin(0.1)
        self.assertGreater(self.samples[-1][1][3], last_wrist + 0.1)


if __name__ == '__main__':
    unittest.main()

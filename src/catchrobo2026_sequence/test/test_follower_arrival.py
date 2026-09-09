"""Check final joint identity with a real follower and synthetic feedback, without CAN."""

import math
import os
import signal
import subprocess
import time

from action_msgs.msg import GoalStatus
from catchrobo2026_msgs.action import FollowRoute
from catchrobo2026_msgs.srv import WristControl
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray

from test_route_handoff import Harness, waypoint_pose


# Reference IK values from the repository's robot_kinematics, in radians.
START_JOINTS = [-0.611352921, 0.403437734, 2.37134409, -2.53023982]
FINAL_JOINTS = {
    10.0: [-0.611352921, 0.419829607, 2.39206767, -2.53023982],
    25.0: [-0.611352921, 0.445065558, 2.42292976, -2.53023982],
}
ZERO_WRIST_JOINTS = [0.0, 0.16085076332092285, 2.430922269821167, 0.0]
START_POSITION = (375.0, 238.0, 186.95)


@pytest.fixture
def arrival_rig(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '229')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node(
        'observer', namespace=f'/arrival_{os.getpid()}_{time.monotonic_ns()}', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    children = []
    log_path = tmp_path / 'nodes.log'
    log = log_path.open('w')
    try:
        yield Harness(node, children, log, executor)
    finally:
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        log.close()
        print(log_path.read_text())


class JointStreams:
    def __init__(self, rig, initial, publish_commands=True):
        self.rig = rig
        self.current_values = list(initial)
        self.current_enabled = True
        self.command_values = list(initial)
        self.command_publications = 0
        self.current = rig.node.create_publisher(Float32MultiArray, 'current_joints', 10)
        self.command = (rig.node.create_publisher(Float32MultiArray, 'target_joint_angles', 10)
                        if publish_commands else None)
        self.timer = rig.node.create_timer(0.02, self.publish)

    def publish(self):
        if self.current_enabled:
            self.current.publish(Float32MultiArray(data=self.current_values))
        if self.command is not None:
            self.command.publish(Float32MultiArray(data=self.command_values))
            self.command_publications += 1

    def close(self):
        self.rig.node.destroy_timer(self.timer)
        self.rig.node.destroy_publisher(self.current)
        if self.command is not None:
            self.rig.node.destroy_publisher(self.command)


def launch_follower(rig, streams):
    rig.launch('nav_director', 'path_follower_node')
    assert rig.follow.wait_for_server(timeout_sec=15)
    rig.until(lambda: streams.current.get_subscription_count() > 0)
    rig.observe(0.15)


def route(positions, yaw=-math.pi):
    path = RosPath()
    path.header.frame_id = 'map'
    path.poses = [PoseStamped(pose=waypoint_pose(position, yaw)) for position in positions]
    return path


def start_route(rig, path, **kwargs):
    handle = rig.resolve(rig.follow.send_goal_async(FollowRoute.Goal(
        start=True, path=path, **kwargs)))
    assert handle.accepted
    return handle.get_result_async()


def assert_succeeded(rig, result):
    wrapped = rig.resolve(result)
    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success


@pytest.mark.parametrize('travel_mm', [10.0, 25.0])
def test_old_joint_commands_cannot_complete_an_unreached_move(arrival_rig, travel_mm):
    rig = arrival_rig
    streams = JointStreams(rig, START_JOINTS)
    destination = (*START_POSITION[:2], START_POSITION[2] - travel_mm)
    try:
        launch_follower(rig, streams)
        result = start_route(rig, route([START_POSITION, destination]))
        rig.until(lambda: rig.targets and
                  abs(rig.targets[-1].pose.position.z * 1000.0 - destination[2]) < 1e-3)
        old_commands = streams.command_publications
        rig.observe(0.45)
        assert streams.command_publications > old_commands
        assert not result.done(), 'Repeated old joint commands falsely completed the move'

        # A correct command is still insufficient while measured joints remain unchanged.
        streams.command_values = list(FINAL_JOINTS[travel_mm])
        rig.observe(0.35)
        assert not result.done(), 'The new command was mistaken for measured arrival'

        streams.current_values = list(FINAL_JOINTS[travel_mm])
        assert_succeeded(rig, result)
    finally:
        streams.close()


def test_measured_final_joints_complete_without_a_joint_command_topic(arrival_rig):
    rig = arrival_rig
    streams = JointStreams(rig, START_JOINTS, publish_commands=False)
    destination = (*START_POSITION[:2], START_POSITION[2] - 10.0)
    try:
        launch_follower(rig, streams)
        result = start_route(rig, route([START_POSITION, destination]))
        rig.until(lambda: bool(rig.targets))
        rig.observe(0.25)
        assert not result.done()
        assert rig.node.count_publishers('target_joint_angles') == 0
        streams.current_values = list(FINAL_JOINTS[10.0])
        assert_succeeded(rig, result)
    finally:
        streams.close()


def test_group_wrist_match_does_not_hide_unreached_arm_joints(arrival_rig):
    rig = arrival_rig
    streams = JointStreams(rig, START_JOINTS)
    requests = []

    def wrist_target(request, response):
        requests.append(request)
        response.success = True
        return response

    service = rig.node.create_service(WristControl, 'wrist_control', wrist_target)
    destination = (*START_POSITION[:2], START_POSITION[2] - 25.0)
    try:
        launch_follower(rig, streams)
        result = start_route(
            rig, route([START_POSITION, destination]), rotation_group_id=1,
            wrist_direction=0, wrist_angles=[START_JOINTS[3], START_JOINTS[3]])
        rig.until(lambda: requests and
                  abs(requests[-1].target.pose.position.z * 1000.0 - destination[2]) < 1e-3)
        rig.observe(0.45)
        assert not result.done(), 'Matching only the wrist and an old command hid arm error'
        streams.command_values = list(FINAL_JOINTS[25.0])
        rig.observe(0.35)
        assert not result.done()
        streams.current_values = list(FINAL_JOINTS[25.0])
        assert_succeeded(rig, result)
    finally:
        streams.close()
        rig.node.destroy_service(service)


def test_final_joints_received_only_before_the_command_do_not_complete(arrival_rig):
    rig = arrival_rig
    streams = JointStreams(rig, FINAL_JOINTS[10.0], publish_commands=False)
    destination = (*START_POSITION[:2], START_POSITION[2] - 10.0)
    try:
        launch_follower(rig, streams)
        # Drain prior feedback before sending a route whose endpoint is already measured.
        streams.current_enabled = False
        rig.observe(0.2)
        result = start_route(rig, route([START_POSITION, destination]))
        rig.until(lambda: bool(rig.targets))
        rig.observe(0.35)
        assert not result.done(), 'Feedback from before the final command completed the route'
        streams.current_enabled = True
        assert_succeeded(rig, result)
    finally:
        streams.close()


def test_normal_arrival_distinguishes_zero_and_minus_full_turn(arrival_rig):
    rig = arrival_rig
    initial = [*ZERO_WRIST_JOINTS[:3], -2.0 * math.pi]
    streams = JointStreams(rig, initial, publish_commands=False)
    try:
        launch_follower(rig, streams)
        # XYZ and wrapped yaw coincide, but the normal command's actual wrist is zero.
        result = start_route(rig, route([(675.0, 200.0, 200.0)] * 2, yaw=0.0))
        rig.until(lambda: bool(rig.targets))
        rig.observe(0.45)
        assert not result.done(), 'Arrival folded a full turn out of the wrist error'
        streams.current_values = list(ZERO_WRIST_JOINTS)
        assert_succeeded(rig, result)
    finally:
        streams.close()

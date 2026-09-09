"""Check measured wrist tolerance without expanding Joy's allowed commands."""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from catchrobo2026_msgs.action import FollowRoute
from catchrobo2026_msgs.srv import PlanRotationGroup, WristControl
from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray

from test_follower_arrival import JointStreams, ZERO_WRIST_JOINTS
from test_route_handoff import Harness, waypoint_pose


@pytest.fixture
def wrist_rig(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '223')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node(
        'observer', namespace=f'/wrist_tolerance_{os.getpid()}_{time.monotonic_ns()}',
        context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    children = []
    log_path = tmp_path / 'nodes.log'
    log = log_path.open('w')
    rig = Harness(node, children, log, executor)
    rig.log_path = log_path
    rig.wrist = node.create_client(WristControl, 'wrist_control')
    rig.commands = []
    rig.subscriptions.append(node.create_subscription(
        Float32MultiArray, 'target_joint_angles',
        lambda message: rig.commands.append((time.monotonic(), list(message.data))), 100))
    rig.streams = JointStreams(rig, ZERO_WRIST_JOINTS, publish_commands=False)
    try:
        yield rig
    finally:
        rig.streams.close()
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


def start_joy(rig, measured_wrist, tolerance=None):
    rig.streams.current_values[3] = measured_wrist
    params = {} if tolerance is None else {'wrist_feedback_tolerance_deg': tolerance}
    rig.launch('catchrobo2026_hand_operated', 'joy_controller_node', params)
    assert rig.wrist.wait_for_service(timeout_sec=15)
    rig.until(lambda: rig.streams.current.get_subscription_count() > 0 and rig.commands)
    rig.observe(0.12)


def request(rig, operation, wrist, *, group_id=1, max_phi_travel=None, target_wrist=None):
    pose_wrist = wrist if target_wrist is None else target_wrist
    target = PoseStamped(pose=waypoint_pose((675, 200, 200), pose_wrist))
    return rig.resolve(rig.wrist.call_async(WristControl.Request(
        operation=operation, group_id=group_id, direction=0, wrist_angle=wrist,
        target=target, limit_phi_travel=max_phi_travel is not None,
        max_phi_travel=0.0 if max_phi_travel is None else max_phi_travel)))


def assert_bounded_hold(rig, wrist):
    start = time.monotonic()
    rig.observe(0.16)
    commands = [values for stamp, values in rig.commands if stamp > start + 0.04]
    assert len(commands) >= 3
    for joints in commands:
        assert joints[:3] == pytest.approx(ZERO_WRIST_JOINTS[:3], abs=2e-6)
        assert all(math.isfinite(value) for value in joints)
        # Preserve the existing Float32 representation of -2pi.
        assert -2.0 * math.pi - 1e-6 <= joints[3] <= 1e-6
        assert joints[3] == pytest.approx(wrist, abs=1e-6)


@pytest.mark.parametrize('side', [1, -1], ids=['above_zero', 'below_minus_full_turn'])
def test_default_tolerance_bounds_start_and_phi_budget(wrist_rig, side):
    rig = wrist_rig
    boundary = 0.0 if side > 0 else -2.0 * math.pi
    start_joy(rig, boundary + side * math.radians(5.9))
    response = request(rig, WristControl.Request.BEGIN, boundary, max_phi_travel=0.0)
    assert response.success, response.message
    assert_bounded_hold(rig, boundary)

    # The first target must compare with the bounded start, not raw overshoot.
    response = request(rig, WristControl.Request.TARGET, boundary)
    assert response.success, response.message
    assert_bounded_hold(rig, boundary)
    response = request(rig, WristControl.Request.END, boundary)
    assert response.success, response.message
    assert_bounded_hold(rig, boundary)


@pytest.mark.parametrize('side', [1, -1], ids=['above_zero', 'below_minus_full_turn'])
def test_feedback_beyond_six_degrees_cannot_be_hidden_by_clamping(wrist_rig, side):
    rig = wrist_rig
    boundary = 0.0 if side > 0 else -2.0 * math.pi
    start_joy(rig, boundary + side * math.radians(6.1), tolerance=6.0)
    response = request(rig, WristControl.Request.BEGIN, boundary)
    assert not response.success
    assert 'Fresh finite current_joints' in response.message


@pytest.mark.parametrize('side', [1, -1], ids=['above_zero', 'below_minus_full_turn'])
def test_zero_tolerance_rejects_overshoot_and_accepts_exact_boundary(wrist_rig, side):
    rig = wrist_rig
    boundary = 0.0 if side > 0 else -2.0 * math.pi
    start_joy(rig, boundary + side * math.radians(0.1), tolerance=0.0)
    response = request(rig, WristControl.Request.BEGIN, boundary)
    assert not response.success
    rig.streams.current_values[3] = boundary
    rig.observe(0.08)
    response = request(rig, WristControl.Request.BEGIN, boundary)
    assert response.success, response.message
    assert_bounded_hold(rig, boundary)


@pytest.mark.parametrize('side', [1, -1], ids=['above_zero', 'below_minus_full_turn'])
def test_feedback_tolerance_does_not_expand_begin_or_target_commands(wrist_rig, side):
    rig = wrist_rig
    boundary = 0.0 if side > 0 else -2.0 * math.pi
    measured = boundary + side * math.radians(5.9)
    start_joy(rig, measured)
    response = request(rig, WristControl.Request.BEGIN, measured, target_wrist=boundary)
    assert not response.success
    response = request(rig, WristControl.Request.BEGIN, boundary)
    assert response.success, response.message
    assert_bounded_hold(rig, boundary)
    response = request(rig, WristControl.Request.TARGET, measured)
    assert not response.success
    assert 'outside [-2pi, 0]' in response.message
    assert_bounded_hold(rig, boundary)


@pytest.mark.parametrize('tolerance', [-0.1, 180.0])
def test_invalid_tolerance_rejects_startup(wrist_rig, tolerance):
    rig = wrist_rig
    child = rig.launch('catchrobo2026_hand_operated', 'joy_controller_node', {
        'wrist_feedback_tolerance_deg': tolerance,
    })
    assert child.wait(timeout=10) != 0
    assert 'wrist_feedback_tolerance_deg must be finite in [0, 180)' in rig.log_path.read_text()


@pytest.mark.parametrize('side', [1, -1], ids=['above_zero', 'below_minus_full_turn'])
def test_navigation_and_joy_recover_from_tolerated_feedback(wrist_rig, side):
    rig = wrist_rig
    boundary = 0.0 if side > 0 else -2.0 * math.pi
    rig.streams.current_values[3] = boundary + side * math.radians(5.9)
    config = Path(get_package_share_directory('nav_director')) / 'config/joint_feedback.yaml'
    assert config.is_file()
    plan_client = rig.node.create_client(PlanRotationGroup, 'plan_rotation_group')
    rig.launch('nav_director', 'path_generator_3d', parameter_files=[config])
    rig.launch('nav_director', 'path_follower_node', parameter_files=[config])
    rig.launch('catchrobo2026_hand_operated', 'joy_controller_node', parameter_files=[config])
    assert plan_client.wait_for_service(timeout_sec=15)
    assert rig.wrist.wait_for_service(timeout_sec=15)
    assert rig.follow.wait_for_server(timeout_sec=15)
    rig.until(lambda: rig.streams.current.get_subscription_count() >= 3 and rig.commands)
    rig.observe(0.15)

    target = waypoint_pose((675, 200, 200), boundary)
    planned = rig.resolve(plan_client.call_async(PlanRotationGroup.Request(
        targets=[target], route_ends=[1], allow_wrist_reversal=True)))
    assert planned.success, planned.message
    assert len(planned.routes) == 1
    path = planned.routes[0]
    assert path.wrist_angles[0] == pytest.approx(boundary, abs=1e-6)
    started = rig.resolve(rig.wrist.call_async(WristControl.Request(
        operation=WristControl.Request.BEGIN, group_id=1, direction=planned.direction,
        wrist_angle=path.wrist_angles[0], target=path.path.poses[0])))
    assert started.success, started.message
    assert_bounded_hold(rig, boundary)

    handle = rig.resolve(rig.follow.send_goal_async(FollowRoute.Goal(
        start=True, path=path.path, rotation_group_id=1, wrist_direction=planned.direction,
        wrist_angles=path.wrist_angles, phi_angles=path.phi_angles)))
    assert handle.accepted
    result = handle.get_result_async()
    rig.observe(0.5)
    assert not result.done(), 'Tolerating feedback must not imply measured arrival'
    assert_bounded_hold(rig, boundary)

    rig.streams.current_values[3] = boundary
    finished = rig.resolve(result)
    assert finished.status == GoalStatus.STATUS_SUCCEEDED
    assert finished.result.success
    assert_bounded_hold(rig, boundary)
    ended = request(rig, WristControl.Request.END, boundary)
    assert ended.success, ended.message

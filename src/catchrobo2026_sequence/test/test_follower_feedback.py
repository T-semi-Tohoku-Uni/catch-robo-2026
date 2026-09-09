"""Distinguish feedback failures with a real follower and synthetic local feedback."""

import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from catchrobo2026_msgs.action import FollowRoute
from catchrobo2026_msgs.srv import WristControl
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

from test_follower_arrival import (
    FINAL_JOINTS, JointStreams, START_JOINTS, START_POSITION, ZERO_WRIST_JOINTS, route,
)
from test_route_handoff import Harness


@pytest.fixture
def feedback_rig(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '232')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node(
        'observer', namespace=f'/feedback_{os.getpid()}_{time.monotonic_ns()}',
        context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    children = []
    log_path = tmp_path / 'nodes.log'
    log = log_path.open('w')
    rig = Harness(node, children, log, executor)
    rig.log_path = log_path
    rig.wrist_requests = []
    rig.streams = JointStreams(rig, START_JOINTS, publish_commands=False)

    def wrist_target(request, response):
        rig.wrist_requests.append(request)
        response.success = True
        return response

    service = node.create_service(WristControl, 'wrist_control', wrist_target)
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
        node.destroy_service(service)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        log.close()
        print(log_path.read_text())


def group_goal():
    destination = (*START_POSITION[:2], START_POSITION[2] - 25.0)
    return FollowRoute.Goal(
        start=True, path=route([START_POSITION, destination]), rotation_group_id=1,
        wrist_direction=0, wrist_angles=[START_JOINTS[3], START_JOINTS[3]])


def launch_follower(rig, streams, tolerance=None):
    config = Path(get_package_share_directory('nav_director')) / 'config/joint_feedback.yaml'
    parameters = {} if tolerance is None else {'wrist_feedback_tolerance_deg': tolerance}
    rig.launch('nav_director', 'path_follower_node', params=parameters,
               parameter_files=(config,))
    assert rig.follow.wait_for_server(timeout_sec=15)
    rig.until(lambda: streams.current.get_subscription_count() > 0)
    rig.observe(0.15)


def start_group(rig, tolerance=None):
    launch_follower(rig, rig.streams, tolerance)
    handle = rig.resolve(rig.follow.send_goal_async(group_goal()))
    assert handle.accepted
    result = handle.get_result_async()
    rig.until(lambda: bool(rig.wrist_requests))
    assert not result.done()
    return result


def diagnostic(rig, reason, phase='during sequence group'):
    prefix = f'current_joints rejected {phase}: reason={reason}'
    output = rig.log_path.read_text()
    assert prefix in output, output
    line = next(line for line in output.splitlines() if prefix in line)
    for field in ('sample_size=', 'last_sample_age_sec=', 'last_valid_age_sec=',
                  'timeout_sec=1.000000', 'q=[', 'wrist_range=['):
        assert field in line, line
    return line


def numeric_field(line, name):
    found = re.search(rf'{name}=([-+0-9.eE]+)', line)
    assert found, line
    return float(found.group(1))


def assert_reported_sample(line, expected):
    found = re.search(r'q=\[([^]]*)\]', line)
    assert found, line
    reported = found.group(1).split(',')
    assert len(reported) == 4
    for index, raw in enumerate(reported):
        if index >= len(expected):
            assert raw == 'missing'
        elif math.isnan(expected[index]):
            assert math.isnan(float(raw))
        elif math.isinf(expected[index]):
            assert float(raw) == expected[index]
        else:
            assert float(raw) == pytest.approx(expected[index], rel=1e-6, abs=1e-8)
    bounds = re.search(r'wrist_range=\[([^]]*)\]', line)
    assert bounds, line
    assert [float(value) for value in bounds.group(1).split(',')] == pytest.approx(
        [-2.0 * math.pi, 0.0])


def assert_aborted(rig, result):
    wrapped = rig.resolve(result)
    assert wrapped.status == GoalStatus.STATUS_ABORTED
    assert not wrapped.result.success
    request_count = len(rig.wrist_requests)
    rig.observe(0.15)
    assert len(rig.wrist_requests) == request_count


def test_repeated_equal_feedback_and_short_gap_remain_valid(feedback_rig):
    rig = feedback_rig
    result = start_group(rig)
    count = len(rig.wrist_requests)
    rig.observe(1.2)
    assert len(rig.wrist_requests) > count + 10
    assert not result.done()

    rig.streams.current_enabled = False
    rig.observe(0.3)
    assert not result.done()
    rig.streams.current_enabled = True
    rig.observe(1.1)
    assert not result.done()
    assert 'current_joints rejected' not in rig.log_path.read_text()

    rig.streams.current_values = list(FINAL_JOINTS[25.0])
    wrapped = rig.resolve(result)
    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success


def test_missing_feedback_rejects_group_start(feedback_rig):
    rig = feedback_rig
    rig.streams.current_enabled = False
    launch_follower(rig, rig.streams)
    handle = rig.resolve(rig.follow.send_goal_async(group_goal()))
    assert not handle.accepted
    assert not rig.wrist_requests
    line = diagnostic(rig, 'not_received', 'at sequence group start')
    assert 'last_sample_age_sec=never' in line
    assert 'last_valid_age_sec=never' in line
    assert_reported_sample(line, [])


def test_feedback_timeout_aborts_active_group(feedback_rig):
    rig = feedback_rig
    result = start_group(rig)
    rig.streams.current_enabled = False
    stopped_at = time.monotonic()
    assert_aborted(rig, result)
    assert time.monotonic() - stopped_at >= 0.8
    line = diagnostic(rig, 'stale')
    assert numeric_field(line, 'sample_size') == 4
    assert numeric_field(line, 'last_sample_age_sec') >= 1.0
    assert numeric_field(line, 'last_valid_age_sec') >= 1.0
    assert_reported_sample(line, START_JOINTS)


@pytest.mark.parametrize('values,reason', [
    ([], 'too_few_values'),
    (START_JOINTS[:3], 'too_few_values'),
    ([START_JOINTS[0], math.nan, *START_JOINTS[2:]], 'non_finite'),
    ([*START_JOINTS[:2], math.inf, START_JOINTS[3]], 'non_finite'),
    ([*START_JOINTS[:3], math.radians(6.1)], 'wrist_out_of_range'),
    ([*START_JOINTS[:3], -2.0 * math.pi - math.radians(6.1)], 'wrist_out_of_range'),
], ids=['empty', 'short', 'nan', 'infinity', 'above_zero', 'below_minus_full_turn'])
def test_invalid_feedback_aborts_active_group(feedback_rig, values, reason):
    rig = feedback_rig
    result = start_group(rig)
    rig.streams.current_values = list(values)
    assert_aborted(rig, result)
    line = diagnostic(rig, reason)
    assert numeric_field(line, 'sample_size') == len(values)
    assert numeric_field(line, 'last_sample_age_sec') < 0.5
    assert_reported_sample(line, values)
    assert numeric_field(line, 'wrist_feedback_tolerance_deg') == 6.0


@pytest.mark.parametrize('boundary', [0.0, -2.0 * math.pi])
def test_configured_feedback_margin_keeps_active_group_running(feedback_rig, boundary):
    rig = feedback_rig
    result = start_group(rig)
    sign = 1 if boundary == 0.0 else -1
    rig.streams.current_values = [*START_JOINTS[:3], boundary + sign * math.radians(5.9)]
    count = len(rig.wrist_requests)
    rig.observe(1.2)
    assert not result.done()
    assert len(rig.wrist_requests) > count + 10
    assert all(-2.0 * math.pi - 1e-6 <= r.wrist_angle <= 1e-6
               for r in rig.wrist_requests)
    assert 'current_joints rejected' not in rig.log_path.read_text()


@pytest.mark.parametrize('boundary', [0.0, -2.0 * math.pi])
def test_zero_configuration_restores_strict_feedback_limit(feedback_rig, boundary):
    rig = feedback_rig
    result = start_group(rig, tolerance=0.0)
    sign = 1 if boundary == 0.0 else -1
    rig.streams.current_values = [*START_JOINTS[:3], boundary + sign * 1e-4]
    assert_aborted(rig, result)
    line = diagnostic(rig, 'wrist_out_of_range')
    assert numeric_field(line, 'wrist_feedback_tolerance_deg') == 0.0


@pytest.mark.parametrize('boundary', [0.0, -2.0 * math.pi])
def test_tolerated_start_preserves_arrival_accuracy_and_command_limits(feedback_rig, boundary):
    rig = feedback_rig
    sign = 1 if boundary == 0.0 else -1
    rig.streams.current_values = [*ZERO_WRIST_JOINTS[:3], boundary + sign * math.radians(5.9)]
    launch_follower(rig, rig.streams)
    goal = FollowRoute.Goal(
        start=True, path=route([(675.0, 200.0, 200.0)] * 2, yaw=0.0),
        rotation_group_id=1, wrist_direction=0, wrist_angles=[boundary, boundary])
    handle = rig.resolve(rig.follow.send_goal_async(goal))
    assert handle.accepted
    result = handle.get_result_async()
    rig.until(lambda: bool(rig.wrist_requests))
    rig.observe(0.4)
    assert not result.done(), 'Feedback tolerance must not relax arrival accuracy'
    assert all(r.wrist_angle == pytest.approx(boundary) for r in rig.wrist_requests)
    rig.streams.current_values = [*ZERO_WRIST_JOINTS[:3], boundary]
    wrapped = rig.resolve(result)
    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success


@pytest.mark.parametrize('boundary', [0.0, -2.0 * math.pi])
def test_measurement_tolerance_does_not_allow_outside_goal_commands(feedback_rig, boundary):
    rig = feedback_rig
    rig.streams.current_values = [*ZERO_WRIST_JOINTS[:3], boundary]
    launch_follower(rig, rig.streams)
    outside = boundary + (1 if boundary == 0.0 else -1) * math.radians(1.0)
    goal = FollowRoute.Goal(
        start=True, path=route([(675.0, 200.0, 200.0)] * 2, yaw=0.0),
        rotation_group_id=1, wrist_direction=0, wrist_angles=[boundary, outside])
    handle = rig.resolve(rig.follow.send_goal_async(goal))
    assert not handle.accepted
    assert not rig.wrist_requests

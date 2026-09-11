"""Exercise waypoint sequencing against gated ROS services/actions, without CAN."""

import copy
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from catchrobo2026_msgs.action import ExecuteSequence, FollowRoute
from catchrobo2026_msgs.srv import EndeffectorControl, GenerateRoute, PumpControl
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
import yaml


@pytest.fixture
def rig(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '228')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('backend', namespace=f'/waypoint_test_{os.getpid()}',
                            context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    log = (tmp_path / 'sequence.log').open('w')
    harness = Harness(node, executor, log, tmp_path)
    try:
        yield harness
    finally:
        for entry in harness.follows:
            if not entry['gate'].done():
                entry['gate'].set_result(False)
        harness.observe(0.1)
        for child in harness.children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in harness.children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        harness.follower.destroy()
        harness.sequence.destroy()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        log.close()
        print((tmp_path / 'sequence.log').read_text())


class Harness:
    def __init__(self, node, executor, log, directory):
        self.node, self.executor, self.log, self.directory = node, executor, log, directory
        self.children, self.plans, self.paths, self.follows = [], [], [], []
        self.pumps, self.endeffectors, self.feedback = [], [], []
        self.plan_mode = 'success'
        self.cancel_count = 0
        self.step_id = 0
        self.services = [
            node.create_service(GenerateRoute, 'generate_route', self.generate),
            node.create_service(PumpControl, 'set_pump_state', self.pump),
            node.create_service(EndeffectorControl, 'set_endeffector_state', self.endeffector),
        ]
        self.follower = ActionServer(
            node, FollowRoute, 'follow_route', self.follow,
            cancel_callback=self.cancel, callback_group=ReentrantCallbackGroup())
        self.sequence = ActionClient(node, ExecuteSequence, 'execute_sequence')

    def generate(self, request, response):
        self.plans.append(copy.deepcopy(request))
        response.success = self.plan_mode != 'reject'
        if self.plan_mode == 'success':
            path = RosPath()
            path.header.frame_id = 'map'
            path.header.stamp.sec = len(self.plans)
            for waypoint in request.waypoints:
                path.poses.append(PoseStamped(pose=copy.deepcopy(waypoint)))
            target = PoseStamped()
            target.pose.position.x = request.x / 1000.0
            target.pose.position.y = request.y / 1000.0
            target.pose.position.z = request.z / 1000.0
            target.pose.orientation.z = math.sin(request.phi / 2.0)
            target.pose.orientation.w = math.cos(request.phi / 2.0)
            path.poses.append(target)
            response.path = path
        self.paths.append(copy.deepcopy(response.path))
        return response

    def pump(self, request, response):
        self.pumps.append(copy.deepcopy(request))
        response.success = True
        return response

    def endeffector(self, request, response):
        self.endeffectors.append(copy.deepcopy(request))
        response.success = True
        return response

    async def follow(self, goal):
        # Yield the executor so action cancellation and other services remain responsive.
        gate = Future(executor=self.executor)
        self.follows.append({'goal': goal, 'gate': gate,
                             'request': copy.deepcopy(goal.request)})
        success = await gate
        if goal.is_cancel_requested:
            goal.canceled()
            success = False
        elif success:
            goal.succeed()
        else:
            goal.abort()
        return FollowRoute.Result(success=success)

    def cancel(self, goal):
        self.cancel_count += 1
        for entry in self.follows:
            if entry['goal'].goal_id == goal.goal_id and not entry['gate'].done():
                entry['gate'].set_result(False)
        return CancelResponse.ACCEPT

    def launch(self, steps, recovery_steps=None, *, sequences=None, poses=None,
               config_options=None, debug=False, route_timeout_sec=15.0,
               sequence_timeout_sec=30.0):
        config = yaml.safe_load(
            (Path(__file__).parents[1] / 'config/sequences.example.yaml').read_text())
        config['sequences'] = {**(sequences or {}), 'tested': {'steps': steps}}
        config['poses'] = poses or {}
        config['end_sequence'] = 'tested'
        config.update(config_options or {})
        if recovery_steps is not None:
            config['sequences']['recovery'] = {'steps': recovery_steps}
            config['start_sequence'] = 'recovery'
        filename = self.directory / 'sequences.yaml'
        self.config_file = filename
        filename.write_text(yaml.safe_dump(config))
        self.launch_file(filename, debug=debug, route_timeout_sec=route_timeout_sec,
                         sequence_timeout_sec=sequence_timeout_sec)

    def launch_file(self, filename, *, debug=False, route_timeout_sec=15.0,
                    sequence_timeout_sec=30.0):
        self.config_file = Path(filename)
        prefix = Path(get_package_prefix('catchrobo2026_sequence'))
        args = [str(prefix / 'lib/catchrobo2026_sequence/sequence_node'), '--ros-args',
                '-r', f'__ns:={self.node.get_namespace()}',
                '-p', 'team:=red', '-p', f'sequence_file:={self.config_file}',
                '-p', f'debug:={str(debug).lower()}',
                '-p', 'service_timeout_sec:=5.0', '-p', f'route_timeout_sec:={route_timeout_sec}',
                '-p', f'sequence_timeout_sec:={sequence_timeout_sec}',
                '-p', 'stop_timeout_sec:=2.0']
        self.children.append(subprocess.Popen(
            args, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True))
        assert self.sequence.wait_for_server(timeout_sec=15)

    def until(self, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert all(p.poll() is None for p in self.children), 'sequence node exited'
            assert time.monotonic() < deadline, 'timed out waiting for ROS response'
            self.executor.spin_once(timeout_sec=0.02)

    def observe(self, duration=0.2):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def resolve(self, future):
        self.until(future.done)
        return future.result()

    def start(self, kind=ExecuteSequence.Goal.END, *, row=0, column=0,
              box=0, box_column=0, collector_mask=7):
        self.step_id += 1
        handle = self.resolve(self.sequence.send_goal_async(ExecuteSequence.Goal(
            control_epoch=1, step_id=self.step_id, kind=kind, row=row, column=column,
            box=box, box_column=box_column, collector_mask=collector_mask),
            feedback_callback=lambda message: self.feedback.append(copy.deepcopy(message.feedback))))
        assert handle.accepted
        return handle, handle.get_result_async()

    def complete_follow(self, index):
        self.follows[index]['gate'].set_result(True)

    def assert_succeeded(self, future):
        result = self.resolve(future)
        assert result.status == GoalStatus.STATUS_SUCCEEDED
        assert result.result.success, result.result.message
        self.observe()


def move(pose, *, waypoint=False, relative=False, waypoints=None):
    value = {'relative' if relative else 'absolute': pose}
    if waypoint:
        value['waypoint'] = True
    if waypoints is not None:
        value['waypoints'] = waypoints
    return {'move': value}


def waypoint_step(pose, *, command='waypoint', relative=False):
    if command == 'move':
        return move(pose, waypoint=True, relative=relative)
    return {'waypoint': {'relative' if relative else 'absolute': pose}}


def assert_target(request, expected):
    assert [request.x, request.y, request.z, request.phi] == pytest.approx(expected)


def assert_waypoint(pose, expected):
    assert [pose.position.x, pose.position.y, pose.position.z] == pytest.approx(
        [value / 1000.0 for value in expected[:3]])
    assert [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] == (
        pytest.approx([0.0, 0.0, math.sin(expected[3] / 2.0), math.cos(expected[3] / 2.0)]))


@pytest.mark.parametrize('waypoint_command', ['move', 'waypoint'])
def test_waypoint_group_waits_for_follow_before_later_steps(rig, waypoint_command):
    waypoint = [675, 200, 300, 0.4]
    terminal = [670, -110, 220, -0.2]
    next_move = [500, 100, 200, 0.0]
    rig.launch([
        waypoint_step(waypoint, command=waypoint_command), move(terminal), {'wait': 0.1},
        {'pump': 'suction'}, {'endeffector': 1}, move(next_move),
    ])
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    rig.observe()
    assert len(rig.plans) == 1
    assert rig.plans[0].use_explicit_waypoints
    assert len(rig.plans[0].waypoints) == 1
    assert_waypoint(rig.plans[0].waypoints[0], waypoint)
    assert_target(rig.plans[0], terminal)
    assert rig.follows[0]['request'].start
    assert rig.follows[0]['request'].path == rig.paths[0]
    assert not result.done()
    assert not rig.pumps and not rig.endeffectors
    assert not any(feedback.phase == 'waiting' for feedback in rig.feedback)

    rig.complete_follow(0)
    rig.until(lambda: len(rig.follows) == 2)
    assert len(rig.plans) == 2
    assert len(rig.pumps) == 1 and len(rig.endeffectors) == 1
    assert [rig.pumps[0].left, rig.pumps[0].center, rig.pumps[0].right] == [
        PumpControl.Request.SUCTION] * 3
    assert rig.endeffectors[0].command == 1
    assert any(feedback.phase == 'waiting' and feedback.step_index == 2
               for feedback in rig.feedback)
    assert rig.plans[1].use_explicit_waypoints
    assert not rig.plans[1].waypoints
    assert_target(rig.plans[1], next_move)
    assert rig.follows[1]['request'].path == rig.paths[1]
    assert not result.done()
    rig.complete_follow(1)
    rig.assert_succeeded(result)
    assert len(rig.plans) == len(rig.follows) == 2


def test_consecutive_regular_moves_remain_separate(rig):
    targets = [[400, 200, 300, 0.0], [600, 100, 220, 0.5]]
    rig.launch([move(target) for target in targets])
    _, result = rig.start()
    for index, target in enumerate(targets):
        rig.until(lambda: len(rig.follows) == index + 1)
        rig.observe()
        assert len(rig.plans) == index + 1
        assert rig.plans[index].use_explicit_waypoints
        assert not rig.plans[index].waypoints
        assert_target(rig.plans[index], target)
        assert not result.done()
        rig.complete_follow(index)
    rig.assert_succeeded(result)
    assert len(rig.plans) == len(rig.follows) == 2


@pytest.mark.parametrize('waypoint_command', ['move', 'waypoint'])
def test_multiple_relative_waypoints_use_fixed_absolute_origin(rig, waypoint_command):
    rig.launch([
        waypoint_step([100, 200, 300, 0.3], command=waypoint_command),
        waypoint_step([10, -20, 30, 0.2], command=waypoint_command, relative=True),
        move([0, 0, -10, -0.4], relative=True),
    ])
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    request = rig.plans[0]
    assert request.use_explicit_waypoints
    assert len(request.waypoints) == 2
    assert_waypoint(request.waypoints[0], [100, 200, 300, 0.3])
    assert_waypoint(request.waypoints[1], [110, 180, 330, 0.5])
    assert_target(request, [100, 200, 290, -0.1])
    assert rig.follows[0]['request'].path == rig.paths[0]
    rig.complete_follow(0)
    rig.assert_succeeded(result)
    assert len(rig.plans) == len(rig.follows) == 1


def test_common_waypoint_and_inline_points_share_route_and_preserve_origins(rig):
    common = [1200, 0, 400, math.pi]
    named = [1150, 0, 380, 1.0]
    array = [1100, -10, 360, 0.8]
    absolute = [1050, -20, 340, 0.6]
    terminal = [900, -100, 250, 0.1]
    rig.launch([
        {'call': 'place_pre_blue_waypoint'},
        move(terminal, waypoints=[
            'named', array, {'absolute': absolute}, {'relative': [-200, -30, -80, -2]},
        ]),
        move([0, 0, -10, 0.2], relative=True, waypoints=[]),
    ], sequences={
        'place_pre_blue_waypoint': {'steps': [waypoint_step(common)]},
    }, poses={'named': named})
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    rig.observe()
    assert len(rig.plans) == len(rig.follows) == 1
    request = rig.plans[0]
    assert request.use_explicit_waypoints
    expected = [common, named, array, absolute, [1000, -30, 320, math.pi - 2]]
    assert len(request.waypoints) == len(expected)
    for actual, point in zip(request.waypoints, expected):
        assert_waypoint(actual, point)
    assert_target(request, terminal)
    assert rig.follows[0]['request'].path == rig.paths[0]
    assert not result.done()

    # Only the final absolute move becomes the next relative move's origin.
    rig.complete_follow(0)
    rig.until(lambda: len(rig.follows) == 2)
    assert len(rig.plans) == 2
    assert rig.plans[1].use_explicit_waypoints
    assert not rig.plans[1].waypoints
    assert_target(rig.plans[1], [900, -100, 240, 0.3])
    rig.complete_follow(1)
    rig.assert_succeeded(result)
    assert len(rig.plans) == len(rig.follows) == 2


def test_cancel_discards_group_and_does_not_issue_later_commands(rig):
    recovery = [350, 150, 250, -0.4]
    rig.launch([
        move([675, 200, 300, 0.0], waypoint=True), move([670, -110, 220, 0.0]),
        {'pump': 'suction'}, {'wait': 0.1}, {'endeffector': 1},
    ], recovery_steps=[move(recovery)])
    handle, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    canceled = rig.resolve(handle.cancel_goal_async())
    assert canceled.goals_canceling
    outcome = rig.resolve(result)
    assert outcome.status == GoalStatus.STATUS_CANCELED
    assert not outcome.result.success
    rig.observe()
    assert rig.cancel_count == 1
    assert not rig.pumps and not rig.endeffectors
    assert not any(feedback.phase == 'waiting' for feedback in rig.feedback)
    assert len(rig.plans) == len(rig.follows) == 1

    _, recovered = rig.start(ExecuteSequence.Goal.START)
    rig.until(lambda: len(rig.follows) == 2)
    assert len(rig.plans) == 2
    assert rig.plans[1].use_explicit_waypoints
    assert not rig.plans[1].waypoints
    assert_target(rig.plans[1], recovery)
    assert rig.follows[1]['request'].path == rig.paths[1]
    rig.complete_follow(1)
    rig.assert_succeeded(recovered)
    assert not rig.pumps and not rig.endeffectors
    assert len(rig.plans) == len(rig.follows) == 2


@pytest.mark.parametrize('plan_mode', ['reject', 'empty'])
def test_failed_generation_never_starts_follow_or_later_commands(rig, plan_mode):
    rig.plan_mode = plan_mode
    rig.launch([
        move([675, 200, 300, 0.0], waypoint=True), move([670, -110, 220, 0.0]),
        {'pump': 'suction'}, {'endeffector': 1},
    ])
    _, result = rig.start()
    outcome = rig.resolve(result)
    assert outcome.status == GoalStatus.STATUS_ABORTED
    assert not outcome.result.success
    assert 'route generation' in outcome.result.message
    rig.observe()
    assert len(rig.plans) == 1
    assert not rig.follows and not rig.pumps and not rig.endeffectors


@pytest.mark.parametrize('debug', [False, True])
def test_config_route_timeout_is_fixed_during_action_and_reloads_between_actions(rig, debug):
    rig.launch([
        move([600, 200, 300, 0.0]), move([670, -110, 220, 0.0]), {'pump': 'suction'},
    ], config_options={'route_timeout_sec': 1.5}, debug=debug, route_timeout_sec=0.15)
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    rig.observe(0.4)
    assert not result.done() and rig.cancel_count == 0

    config = yaml.safe_load(rig.config_file.read_text())
    config['route_timeout_sec'] = 0.1
    rig.config_file.write_text(yaml.safe_dump(config))
    rig.complete_follow(0)
    rig.until(lambda: len(rig.follows) == 2)
    rig.observe(0.4)
    assert not result.done() and rig.cancel_count == 0
    rig.complete_follow(1)
    rig.assert_succeeded(result)
    assert len(rig.pumps) == 1

    _, next_result = rig.start()
    rig.until(lambda: len(rig.follows) == 3)
    if debug:
        outcome = rig.resolve(next_result)
        assert outcome.status == GoalStatus.STATUS_ABORTED
        assert 'following route timeout' in outcome.result.message
        assert rig.cancel_count == 1 and len(rig.plans) == 3
        assert len(rig.pumps) == 1
    else:
        rig.observe(0.4)
        assert not next_result.done() and rig.cancel_count == 0
        rig.complete_follow(2)
        rig.until(lambda: len(rig.follows) == 4)
        rig.complete_follow(3)
        rig.assert_succeeded(next_result)


def test_missing_config_route_timeout_uses_ros_parameter(rig):
    rig.launch([move([670, -110, 220, 0.0]), {'pump': 'suction'}], route_timeout_sec=0.15)
    _, result = rig.start()
    outcome = rig.resolve(result)
    assert outcome.status == GoalStatus.STATUS_ABORTED
    assert 'following route timeout' in outcome.result.message
    assert rig.cancel_count == 1 and len(rig.follows) == 1
    assert not rig.pumps

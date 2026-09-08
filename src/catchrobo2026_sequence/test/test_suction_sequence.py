"""Exercise suction checks and sequence control flow with isolated ROS mocks."""

from collections import deque
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from catchrobo2026_msgs.action import ExecuteSequence, FollowRoute
from catchrobo2026_msgs.srv import (
    CheckSuction, EndeffectorControl, GenerateRoute, PumpControl,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from std_msgs.msg import Int32MultiArray
import yaml


@dataclass
class CheckReply:
    success: bool
    timed_out: bool = False
    delay: float = 0.0
    hold: bool = False


@pytest.fixture
def rig(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '228')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = rclpy.context.Context()
    rclpy.init(context=context)
    namespace = f'/suction_test_{os.getpid()}_{time.monotonic_ns()}'
    node = rclpy.create_node('mocks', namespace=namespace, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    log_path = tmp_path / 'sequence.log'
    log = log_path.open('w')
    harness = Harness(node, executor, tmp_path, log)
    try:
        yield harness
    finally:
        for child in harness.children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in harness.children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        for timer in harness.timers:
            timer.cancel()
        executor.shutdown()
        harness.sequence.destroy()
        harness.follower.destroy()
        node.destroy_node()
        context.shutdown()
        log.close()
        print(log_path.read_text())


class Harness:
    def __init__(self, node, executor, tmp_path, log):
        self.node, self.executor, self.tmp_path, self.log = node, executor, tmp_path, log
        self.children, self.timers, self.events = [], [], []
        self.checks, self.replied, self.pumps, self.widths, self.routes = [], [], [], [], []
        self.check_replies, self.held = deque(), {}
        self.step_id = 0
        self.group = ReentrantCallbackGroup()
        self.services = [
            node.create_service(GenerateRoute, 'generate_route', self.generate,
                                callback_group=self.group),
            node.create_service(PumpControl, 'set_pump_state', self.pump,
                                callback_group=self.group),
            node.create_service(EndeffectorControl, 'set_endeffector_state', self.endeffector,
                                callback_group=self.group),
            node.create_service(CheckSuction, 'check_suction', self.check,
                                callback_group=self.group),
        ]
        self.follower = ActionServer(
            node, FollowRoute, 'follow_route', self.follow,
            cancel_callback=lambda goal: CancelResponse.ACCEPT, callback_group=self.group)
        self.sequence = ActionClient(node, ExecuteSequence, 'execute_sequence')

    def generate(self, request, response):
        target = (request.x, request.y, request.z, request.phi)
        self.events.append(('move', target))
        self.routes.append(target)
        pose = PoseStamped()
        pose.pose.position.x = request.x / 1000
        pose.pose.position.y = request.y / 1000
        pose.pose.position.z = request.z / 1000
        pose.pose.orientation.w = 1.0
        response.path = RosPath(poses=[pose])
        response.success = True
        return response

    def follow(self, goal):
        assert goal.request.start and goal.request.path.poses
        goal.succeed()
        return FollowRoute.Result(success=True)

    def pump(self, request, response):
        states = (request.left, request.center, request.right)
        self.events.append(('pump', states))
        self.pumps.append(states)
        response.success = True
        return response

    def endeffector(self, request, response):
        self.events.append(('endeffector', request.command))
        self.widths.append(request.command)
        response.success = True
        return response

    async def check(self, request, response):
        index = len(self.checks)
        self.events.append(('check_start', index))
        self.checks.append((request.collector_mask, request.timeout_sec))
        assert self.check_replies, 'unexpected suction retry'
        reply = self.check_replies.popleft()
        if reply.hold:
            pending = Future(executor=self.executor)
            self.held[index] = pending
            await pending
        elif reply.delay:
            pending = Future(executor=self.executor)

            def complete():
                timer.cancel()
                pending.set_result(True)

            timer = self.node.create_timer(reply.delay, complete, callback_group=self.group)
            self.timers.append(timer)
            await pending
        response.success = reply.success
        response.timed_out = reply.timed_out
        response.suction_mask = request.collector_mask if reply.success else 0
        response.pressure = [0x60 if reply.success else 0x45] * 3
        response.message = 'mock acquired' if reply.success else 'mock timeout or rejection'
        self.replied.append(index)
        self.events.append(('check_reply', index))
        return response

    def launch(self, steps, **params):
        config = {
            'version': 1,
            'poses': {},
            'sequences': {'pick': {'steps': steps}},
            'bindings': {
                'red': {'pick': {'0,1': 'pick'}, 'place': {}},
                'blue': {'pick': {}, 'place': {}},
            },
        }
        config_file = self.tmp_path / 'sequences.yaml'
        config_file.write_text(yaml.safe_dump(config))
        self.launch_file(config_file, **params)

    def launch_file(self, config_file, **params):
        parameters = {
            'team': 'red', 'sequence_file': config_file,
            'service_timeout_sec': 1.0, 'stop_timeout_sec': 0.3,
            'route_timeout_sec': 2.0, 'sequence_timeout_sec': 20.0,
        }
        parameters.update(params)
        self.launch_node('catchrobo2026_sequence', 'sequence_node', parameters)
        assert self.sequence.wait_for_server(timeout_sec=10)
        self.observe(0.2)

    def launch_node(self, package, executable, parameters):
        binary = Path(get_package_prefix(package)) / 'lib' / package / executable
        args = [str(binary), '--ros-args', '-r', f'__ns:={self.node.get_namespace()}']
        for key, value in parameters.items():
            args += ['-p', f'{key}:={value}']
        child = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        self.children.append(child)

    def until(self, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert all(child.poll() is None for child in self.children), 'sequence node exited'
            assert time.monotonic() < deadline, 'timed out waiting for ROS response'
            self.executor.spin_once(timeout_sec=0.01)

    def observe(self, duration=0.1):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def resolve(self, future, timeout=8):
        self.until(future.done, timeout)
        return future.result()

    def start(self, mask=7):
        self.step_id += 1
        goal = ExecuteSequence.Goal(
            control_epoch=1, step_id=self.step_id, kind=ExecuteSequence.Goal.PICK,
            row=0, column=1, collector_mask=mask)
        handle = self.resolve(self.sequence.send_goal_async(goal))
        assert handle.accepted
        return handle, handle.get_result_async()

    def release(self, index):
        self.held.pop(index).set_result(True)


def retry_steps(timeout=0.6):
    return [
        {'move': {'absolute': [100, 200, 300, 0]}},
        {'for': {'max_iterations': 3, 'steps': [
            {'suction_check': {'start': {'timeout': timeout}}},
            {'pump': 'suction'},
            {'move': {'relative': [0, 0, -10, 0]}},
            {'move': {'relative': [0, 0, 10, 0]}},
            {'suction_check': 'wait'},
            {'if': {'condition': 'suction_success', 'then': [{'break': True}]}},
        ]}},
        {'if': {'condition': 'suction_failure',
                'then': [{'fail': 'suction retry limit reached'}]}},
        {'endeffector': 1},
    ]


@pytest.mark.parametrize('outcomes, attempts, success', [
    ([True], 1, True),
    ([False, True], 2, True),
    ([False, False, False], 3, False),
])
def test_pick_retries_break_on_success_and_fail_at_limit(rig, outcomes, attempts, success):
    rig.check_replies.extend(CheckReply(value, timed_out=not value) for value in outcomes)
    rig.launch(retry_steps())
    _, pending = rig.start(mask=5)
    result = rig.resolve(pending)
    assert result.result.success == success, result.result.message
    assert result.status == (GoalStatus.STATUS_SUCCEEDED if success else GoalStatus.STATUS_ABORTED)
    assert rig.checks == [(5, 0.6)] * attempts
    assert rig.pumps == [(1, 0, 1)] * attempts
    assert rig.routes == [(100, 200, 300, 0)] + [
        (100, 200, 290, 0), (100, 200, 310, 0)] * attempts
    assert rig.widths == ([1] if success else [])
    if not success:
        assert 'suction retry limit reached' in result.result.message
    starts = [i for i, event in enumerate(rig.events) if event[0] == 'check_start']
    pumps = [i for i, event in enumerate(rig.events) if event[0] == 'pump']
    assert all(start < pump for start, pump in zip(starts, pumps))


def test_monitor_runs_while_pump_and_routes_execute_then_waits(rig):
    rig.check_replies.append(CheckReply(True, hold=True))
    rig.launch(retry_steps(timeout=2.0))
    _, pending = rig.start()
    rig.until(lambda: len(rig.routes) == 3 and len(rig.pumps) == 1)
    rig.observe(0.1)
    assert not pending.done()
    assert rig.replied == [] and rig.widths == []
    rig.release(0)
    assert rig.resolve(pending).result.success
    assert len(rig.checks) == 1 and rig.widths == [1]


def test_builtin_pick_common_retries_with_configured_moves(rig):
    config_file = Path(__file__).parents[1] / 'config/sequences.yaml'
    config = yaml.load(config_file.read_text(), Loader=yaml.BaseLoader)
    values = config['values']
    timeout = float(values['pick_suction_timeout_sec'])
    above = tuple(float(values[value[1:]]) if value.startswith('$') else float(value)
                  for value in config['poses']['pick_0_1_above'])
    approach = (*above[:2], above[2] + float(values['pick_approach_dz']), above[3])
    retreat = (*above[:2], above[2] + float(values['pick_retreat_dz']), above[3])
    rig.check_replies.extend([
        CheckReply(False, timed_out=True, delay=timeout), CheckReply(True),
    ])
    rig.launch_file(config_file)
    _, pending = rig.start(mask=3)
    result = rig.resolve(pending)
    assert result.result.success, result.result.message
    assert rig.checks == [(3, timeout)] * 2
    assert rig.pumps == [(1, 1, 0)] * 2
    assert rig.routes == [above, approach, retreat, approach, retreat]
    assert rig.widths == [int(values['place_endeffector_command'])]


def test_real_pump_sensor_timeout_then_selected_collectors_succeed(rig):
    # Replace only the pump mocks; all motion remains isolated and simulated.
    for service in (rig.services[1], rig.services[3]):
        rig.node.destroy_service(service)
    pressure = rig.node.create_publisher(
        Int32MultiArray, 'pressure_sensor', qos_profile_sensor_data)
    outputs = []
    rig.node.create_subscription(Int32MultiArray, 'pump_state',
                                 lambda message: outputs.append(message.data[0]), 100)
    readings = [0, 0, 0]
    timer = rig.node.create_timer(
        0.02, lambda: pressure.publish(Int32MultiArray(data=readings)))
    rig.timers.append(timer)
    rig.launch_node('catchrobo2026_pump', 'pump_controller_node', {
        'pressure_comparison': 'ge', 'initial_state': 0,
    })
    rig.until(lambda: pressure.get_subscription_count() == 1 and bool(outputs))
    rig.launch(retry_steps(timeout=0.6))
    _, pending = rig.start(mask=5)
    rig.until(lambda: len(rig.routes) == 4)
    assert not pending.done()
    readings[:] = [0x60, 0, 0x60]
    result = rig.resolve(pending)
    assert result.result.success, result.result.message
    assert len(rig.routes) == 5 and rig.widths == [1]
    assert len(outputs) >= 30 and outputs[-1] == 58


def test_all_collector_masks_are_forwarded_and_other_pumps_are_off(rig):
    rig.check_replies.extend(CheckReply(True) for _ in range(7))
    rig.launch([
        {'suction_check': {'start': {'timeout': 0.4}}},
        {'pump': 'suction'},
        {'suction_check': 'wait'},
    ])
    for mask in range(1, 8):
        _, pending = rig.start(mask)
        assert rig.resolve(pending).result.success
        assert rig.checks[-1] == (mask, 0.4)
        assert rig.pumps[-1] == tuple(1 if mask & bit else 0 for bit in (1, 2, 4))
    assert len(rig.checks) == len(rig.pumps) == 7


def test_if_else_uses_the_latest_completed_suction_result(rig):
    rig.check_replies.extend([CheckReply(True), CheckReply(False, timed_out=True)])
    rig.launch([
        {'suction_check': {'start': {'timeout': 0.4}}},
        {'suction_check': 'wait'},
        {'if': {'condition': 'suction_success', 'then': [{'pump': 'suction'}],
                'else': [{'pump': 'off'}]}},
        {'if': {'condition': 'suction_failure', 'then': [{'endeffector': 0}],
                'else': [{'endeffector': 1}]}},
    ])
    for _ in range(2):
        _, pending = rig.start()
        assert rig.resolve(pending).result.success
    assert rig.pumps == [(1, 1, 1), (0, 0, 0)]
    assert rig.widths == [1, 0]


def test_nested_break_exits_only_the_innermost_for(rig):
    rig.launch([
        {'for': {'max_iterations': 3, 'steps': [
            {'pump': 'suction'},
            {'for': {'max_iterations': 5, 'steps': [
                {'endeffector': 0}, {'break': True}, {'endeffector': 1},
            ]}},
            {'pump': 'release'},
        ]}},
        {'endeffector': 1},
    ])
    _, pending = rig.start()
    assert rig.resolve(pending).result.success
    assert rig.pumps == [(1, 1, 1), (-1, -1, -1)] * 3
    assert rig.widths == [0, 0, 0, 1]
    assert rig.checks == []


def test_cancel_detaches_monitor_and_late_reply_cannot_change_next_action(rig):
    rig.check_replies.extend([
        CheckReply(True, hold=True), CheckReply(False, timed_out=True, hold=True),
    ])
    rig.launch([
        {'suction_check': {'start': {'timeout': 2.0}}},
        {'pump': 'suction'},
        {'suction_check': 'wait'},
        {'if': {'condition': 'suction_success', 'then': [{'endeffector': 1}],
                'else': [{'endeffector': 0}]}},
    ])
    first, first_result = rig.start()
    rig.until(lambda: len(rig.pumps) == 1 and len(rig.held) == 1)
    assert rig.resolve(first.cancel_goal_async()).goals_canceling
    canceled = rig.resolve(first_result, timeout=1)
    assert not canceled.result.success and canceled.status == GoalStatus.STATUS_CANCELED
    assert rig.replied == []

    _, second_result = rig.start()
    rig.until(lambda: len(rig.pumps) == 2 and len(rig.held) == 2)
    rig.release(0)
    rig.until(lambda: rig.replied == [0])
    rig.observe(0.15)
    assert not second_result.done() and rig.widths == []
    rig.release(1)
    assert rig.resolve(second_result).result.success
    assert rig.widths == [0]
    assert len(rig.checks) == 2


def test_suction_deadline_includes_sensor_timeout_and_service_grace(rig):
    rig.check_replies.append(CheckReply(True, delay=0.55))
    rig.launch([
        {'suction_check': {'start': {'timeout': 0.9}}},
        {'pump': 'suction'},
        {'suction_check': 'wait'},
        {'if': {'condition': 'suction_failure', 'then': [{'fail': 'unexpected failure'}]}},
        {'endeffector': 1},
    ], service_timeout_sec=0.25)
    started = time.monotonic()
    _, pending = rig.start()
    result = rig.resolve(pending)
    assert result.result.success, result.result.message
    assert time.monotonic() - started >= 0.5
    assert rig.widths == [1] and rig.pumps == [(1, 1, 1)]


@pytest.mark.parametrize('reply', [CheckReply(False), CheckReply(True, hold=True)])
def test_rejected_or_missing_suction_reply_aborts_without_retry(rig, reply):
    rig.check_replies.append(reply)
    rig.launch(retry_steps(timeout=0.2), service_timeout_sec=0.4)
    _, pending = rig.start()
    result = rig.resolve(pending)
    assert not result.result.success and result.status == GoalStatus.STATUS_ABORTED
    assert len(rig.checks) == 1 and len(rig.pumps) <= 1
    assert rig.widths == []
    assert 'suction' in result.result.message.lower()

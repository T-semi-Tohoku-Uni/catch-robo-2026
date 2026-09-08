"""Test lifecycle moves with real navigation nodes and an ideal robot, without CAN."""

import copy
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from catchrobo2026_msgs.action import ExecuteSequence, FollowRoute
from catchrobo2026_msgs.srv import GenerateRoute
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from std_srvs.srv import Trigger
import yaml


@pytest.fixture
def rig(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '227')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('observer', namespace=f'/handoff_test_{os.getpid()}', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    children = []
    log = (tmp_path / 'nodes.log').open('w')
    try:
        harness = Harness(node, children, log, executor)
        yield harness
    finally:
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGCONT)
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
        print((tmp_path / 'nodes.log').read_text())


class Harness:
    def __init__(self, node, children, log, executor):
        self.node, self.children, self.log = node, children, log
        self.executor = executor
        self.poses, self.targets, self.routes, self.initializations = [], [], [], []
        self.subscriptions = [
            node.create_subscription(PoseStamped, 'current_pose', self.poses.append, 10),
            node.create_subscription(PoseStamped, 'target_pose', self.targets.append, 10),
            node.create_subscription(RosPath, 'route', self.routes.append, 10),
        ]
        self.route_pub = node.create_publisher(RosPath, 'follower_route', 10)
        self.trigger = node.create_service(Trigger, 'request_initialization', self.initialize)
        self.sequence = ActionClient(node, ExecuteSequence, 'execute_sequence')
        self.follow = ActionClient(node, FollowRoute, 'follow_route')
        self.generate = node.create_client(GenerateRoute, 'generate_route')
        self.step_id = 0

    def initialize(self, request, response):
        self.initializations.append((copy.deepcopy(self.poses[-1]), len(self.routes)))
        response.success = True
        return response

    def launch(self, package, executable, params=None, remaps=()):
        prefix = Path(get_package_prefix(package))
        args = [str(prefix / 'lib' / package / executable), '--ros-args',
                '-r', f'__ns:={self.node.get_namespace()}']
        for remap in remaps:
            args += ['-r', remap]
        for key, value in (params or {}).items():
            args += ['-p', f'{key}:={value}']
        child = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        self.children.append(child)
        return child

    def until(self, predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert all(p.poll() is None for p in self.children), 'node exited'
            assert time.monotonic() < deadline, 'timed out waiting for ROS response'
            self.executor.spin_once(timeout_sec=0.02)

    def observe(self, duration=0.3):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def resolve(self, future):
        self.until(future.done)
        return future.result()

    def execute(self, kind):
        self.step_id += 1
        handle = self.resolve(self.sequence.send_goal_async(ExecuteSequence.Goal(
            control_epoch=1, step_id=self.step_id, kind=kind, collector_mask=7)))
        assert handle.accepted
        result = self.resolve(handle.get_result_async())
        assert result.result.success, result.result.message
        self.observe()


def xyz(pose):
    p = pose.pose.position
    return (1000 * p.x, 1000 * p.y, 1000 * p.z)


def assert_at(pose, expected):
    assert math.dist(xyz(pose), expected) <= 31.0, (xyz(pose), expected)


def test_lifecycle_route_handoff(rig, tmp_path):
    before, after, ending = (450, 100, 200), (670, -110, 220), (550, 200, 200)
    # Preserve strings such as pump "off" across the YAML 1.1 round trip.
    config = yaml.load((Path(__file__).parents[1] / 'config/sequences.yaml').read_text(),
                       Loader=yaml.BaseLoader)
    for name, position in [('before', before), ('after', after), ('ending', ending)]:
        config['poses'][name] = [*position, 0]
        config['sequences'][name] = {'steps': [{'move': {'absolute': name}}]}
    config['before_initialization_sequence'] = 'before'
    config['after_initialization_sequence'] = 'after'
    config['end_sequence'] = 'ending'
    config_file = tmp_path / 'sequences.yaml'
    config_file.write_text(yaml.safe_dump(config))
    rig.launch('nav_director', 'path_generator_3d')
    # No generated route is relayed: execution must use the service response.
    rig.launch('nav_director', 'path_follower_node', remaps=('route:=follower_route',))
    dummy = rig.launch('nav_director', 'dummy_robot_node')
    rig.launch('catchrobo2026_hand_operated', 'joy_controller_node')
    rig.launch('catchrobo2026_sequence', 'sequence_node', {
        'team': 'red', 'sequence_file': config_file, 'route_timeout_sec': 12.0})
    assert rig.sequence.wait_for_server(timeout_sec=15)
    assert rig.follow.wait_for_server(timeout_sec=15)
    assert rig.generate.wait_for_service(timeout_sec=15)
    rig.until(lambda: len(rig.poses) >= 10 and rig.route_pub.get_subscription_count() > 0)
    rig.observe()

    empty = rig.resolve(rig.follow.send_goal_async(FollowRoute.Goal(start=True)))
    assert not empty.accepted
    invalid = RosPath(poses=[PoseStamped()])
    invalid.poses[0].pose.orientation.w = 0.0
    zero_quaternion = rig.resolve(rig.follow.send_goal_async(
        FollowRoute.Goal(start=True, path=invalid)))
    assert not zero_quaternion.accepted
    invalid.poses[0].pose.orientation.w = 1.0
    invalid.poses[0].pose.position.x = float('nan')
    nonfinite = rig.resolve(rig.follow.send_goal_async(
        FollowRoute.Goal(start=True, path=invalid)))
    assert not nonfinite.accepted

    # Seed an already reached route to reproduce the former false success.
    stale = RosPath()
    stale.header.frame_id = 'map'
    stale.poses = [copy.deepcopy(rig.poses[-1])]
    rig.route_pub.publish(stale)
    rig.observe()
    for cycle in range(3):
        start_routes = len(rig.routes)
        start_targets = len(rig.targets)
        rig.execute(ExecuteSequence.Goal.INITIALIZE)
        assert len(rig.initializations) == cycle + 1
        trigger_pose, trigger_routes = rig.initializations[-1]
        assert_at(trigger_pose, before)
        assert trigger_routes == start_routes + 1
        assert len(rig.routes) == start_routes + 2
        assert_at(rig.routes[-2].poses[-1], before)
        assert_at(rig.routes[-1].poses[-1], after)
        assert_at(rig.poses[-1], after)
        assert len(rig.targets) > start_targets
        start_targets = len(rig.targets)
        rig.execute(ExecuteSequence.Goal.END)
        assert len(rig.routes) == start_routes + 3
        assert_at(rig.routes[-1].poses[-1], ending)
        assert_at(rig.poses[-1], ending)
        assert len(rig.targets) > start_targets

    # Freeze feedback so the active goal cannot finish before competing requests.
    response = rig.resolve(rig.generate.call_async(GenerateRoute.Request(
        x=float(after[0]), y=float(after[1]), z=float(after[2]), phi=0.0)))
    assert response.success and response.path.poses
    os.killpg(dummy.pid, signal.SIGSTOP)
    try:
        goal = FollowRoute.Goal(start=True, path=response.path)
        active = rig.resolve(rig.follow.send_goal_async(goal))
        assert active.accepted
        result = active.get_result_async()
        overlap = rig.resolve(rig.follow.send_goal_async(goal))
        assert not overlap.accepted
        # A newly received, already reached route must not replace the active goal.
        stale.poses = [copy.deepcopy(rig.poses[-1])]
        for _ in range(3):
            rig.route_pub.publish(stale)
            rig.observe(0.1)
        assert not result.done()
    finally:
        os.killpg(dummy.pid, signal.SIGCONT)
    assert rig.resolve(result).result.success
    rig.observe()
    assert_at(rig.poses[-1], after)

    # Legacy start-only goals snapshot the route at acceptance and support cancel.
    response = rig.resolve(rig.generate.call_async(GenerateRoute.Request(
        x=float(before[0]), y=float(before[1]), z=float(before[2]), phi=0.0)))
    assert response.success
    rig.route_pub.publish(response.path)
    rig.observe()
    os.killpg(dummy.pid, signal.SIGSTOP)
    try:
        legacy = rig.resolve(rig.follow.send_goal_async(FollowRoute.Goal(start=True)))
        assert legacy.accepted
        legacy_result = legacy.get_result_async()
        stale.poses = [copy.deepcopy(rig.poses[-1])]
        rig.route_pub.publish(stale)
        rig.observe()
        assert not legacy_result.done()
        canceled = rig.resolve(legacy.cancel_goal_async())
        assert canceled.goals_canceling
        assert not rig.resolve(legacy_result).result.success
    finally:
        os.killpg(dummy.pid, signal.SIGCONT)
    rig.observe()
    # A subsequent move must be accepted after cancellation has terminated.
    rig.execute(ExecuteSequence.Goal.END)
    assert_at(rig.poses[-1], ending)

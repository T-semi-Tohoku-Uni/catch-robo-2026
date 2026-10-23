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
from catchrobo2026_msgs.msg import RotationGroupRoute
from catchrobo2026_msgs.srv import (
    CheckSuction, EndeffectorControl, GenerateRoute, PlanRotationGroup, PumpControl, WristControl,
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
from std_msgs.msg import Int32MultiArray, Float32MultiArray
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
        if harness.follow_waiter is not None and not harness.follow_waiter.done():
            harness.follow_waiter.set_result(True)
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
        self.route_requests = []
        self.group_plans, self.wrist_operations, self.follow_goals = [], [], []
        self.check_replies, self.held = deque(), {}
        self.step_id = 0
        self.hold_follow = False
        self.follow_waiter = None
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
            node.create_service(PlanRotationGroup, 'plan_rotation_group', self.plan_group,
                                callback_group=self.group),
            node.create_service(WristControl, 'wrist_control', self.wrist,
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
        self.route_requests.append(request)
        pose = PoseStamped()
        pose.pose.position.x = request.x / 1000
        pose.pose.position.y = request.y / 1000
        pose.pose.position.z = request.z / 1000
        pose.pose.orientation.w = 1.0
        response.path = RosPath(poses=[pose])
        response.success = True
        return response

    async def follow(self, goal):
        assert goal.request.start and goal.request.path.poses
        self.follow_goals.append(goal.request)
        if self.hold_follow:
            self.follow_waiter = Future(executor=self.executor)
            await self.follow_waiter
        if goal.is_cancel_requested:
            goal.canceled()
            return FollowRoute.Result(success=False)
        goal.succeed()
        return FollowRoute.Result(success=True)

    def plan_group(self, request, response):
        self.events.append(('group_plan', len(self.group_plans)))
        self.group_plans.append(request)
        begin = 0
        for end in request.route_ends:
            poses = [PoseStamped(pose=pose) for pose in request.targets[begin:end]]
            response.routes.append(RotationGroupRoute(
                path=RosPath(poses=poses), wrist_angles=[0.0] * len(poses),
                phi_angles=[0.0] * len(poses)))
            begin = end
        response.direction = 0 if request.allow_wrist_reversal else 1
        response.interval_phi_travel = [0.0] * len(request.phi_travel_intervals)
        response.success = True
        return response

    def wrist(self, request, response):
        self.events.append(('wrist', request.operation))
        self.wrist_operations.append(request)
        response.success = True
        return response

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
            'start_sequence': 'lifecycle',
            'sequences': {'lifecycle': {'steps': steps}},
            'bindings': {
                'red': {'pick': {}, 'place': {}},
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
            control_epoch=1, step_id=self.step_id, kind=ExecuteSequence.Goal.START,
            row=0, column=1, collector_mask=mask)
        handle = self.resolve(self.sequence.send_goal_async(goal))
        assert handle.accepted
        return handle, handle.get_result_async()

    def release(self, index):
        self.held.pop(index).set_result(True)

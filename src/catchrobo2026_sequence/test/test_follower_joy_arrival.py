"""Check ordinary route completion through real Joy quaternion/Float32 conversion."""

import copy
import math
import os
import signal
import subprocess
import time

from action_msgs.msg import GoalStatus
from catchrobo2026_msgs.action import FollowRoute
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray
from test_route_handoff import Harness, waypoint_pose, xyz


@pytest.fixture
def arrival_rig(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '230')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    namespace = f'/follower_joy_arrival_{os.getpid()}_{time.monotonic_ns()}'
    node = rclpy.create_node('observer', namespace=namespace, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    children = []
    log_path = tmp_path / 'nodes.log'
    log = log_path.open('w')
    try:
        rig = Harness(node, children, log, executor)
        rig.commands, rig.joints = [], []
        rig.feedback_wrist = None
        joint_publisher = node.create_publisher(Float32MultiArray, 'current_joints', 10)

        def echo_latest_command():
            if not rig.commands:
                return
            joints = list(rig.commands[-1].data)
            if rig.feedback_wrist is not None:
                joints[3] = rig.feedback_wrist
            joint_publisher.publish(Float32MultiArray(data=joints))

        # Echo all four physical joint values without wrapping the wrist angle.
        rig.subscriptions.extend([
            node.create_subscription(
                Float32MultiArray, 'target_joint_angles', rig.commands.append, 100),
            node.create_subscription(
                Float32MultiArray, 'current_joints', rig.joints.append, 100),
        ])
        timer = node.create_timer(0.02, echo_latest_command)
        rig.launch('nav_director', 'path_follower_node',
                   remaps=('route:=follower_route',))
        rig.launch('catchrobo2026_hand_operated', 'joy_controller_node')
        assert rig.follow.wait_for_server(timeout_sec=15)
        rig.until(lambda: len(rig.commands) >= 5 and len(rig.joints) >= 5)
        yield rig
        node.destroy_timer(timer)
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


def joint_match(actual, expected, tolerance=1e-6):
    return len(actual) == len(expected) == 4 and all(
        math.isfinite(value) and abs(value - target) <= tolerance
        for value, target in zip(actual, expected))


@pytest.mark.parametrize(
    'x, phi, expected_wrist, hold_equivalent_wrist',
    [
        pytest.param(675.0, 0.0, 0.0, False, id='axis-zero'),
        pytest.param(675.0, math.pi, -math.pi, False, id='axis-positive-pi'),
        pytest.param(675.0, -math.pi, -math.pi, False, id='axis-negative-pi'),
        pytest.param(675.0, math.tau, 0.0, False, id='axis-positive-full-turn'),
        pytest.param(675.0, -math.tau, -math.tau, True, id='axis-negative-full-turn'),
        pytest.param(674.0, 0.0, -math.tau + math.atan2(1.0, 390.0), False,
                     id='left-of-axis'),
        pytest.param(676.0, 0.0, -math.atan2(1.0, 390.0), False,
                     id='right-of-axis'),
    ],
)
def test_ordinary_endpoint_matches_joy_joint_command(
        arrival_rig, x, phi, expected_wrist, hold_equivalent_wrist):
    rig = arrival_rig
    endpoint = (x, 200.0, 200.0)
    expected_base = math.atan2(x - 675.0, 390.0)
    target_start, command_start = len(rig.targets), len(rig.commands)
    if hold_equivalent_wrist:
        # Zero and -2pi have the same Cartesian pose but distinct physical turns.
        rig.feedback_wrist = 0.0

    path = Path()
    path.header.frame_id = 'map'
    path.poses = [PoseStamped(pose=waypoint_pose(endpoint, phi))]
    handle = rig.resolve(rig.follow.send_goal_async(FollowRoute.Goal(start=True, path=path)))
    assert handle.accepted
    result = handle.get_result_async()

    rig.until(lambda: len(rig.targets) > target_start)
    final_target = copy.deepcopy(rig.targets[-1])
    assert math.dist(xyz(final_target), endpoint) < 1e-3
    orientation = final_target.pose.orientation
    norm = sum(value * value for value in (
        orientation.x, orientation.y, orientation.z, orientation.w))
    encoded_yaw = math.atan2(
        -2 * (orientation.x * orientation.y - orientation.z * orientation.w) / norm,
        1 - 2 * (orientation.x**2 + orientation.z**2) / norm)
    assert abs(math.remainder(encoded_yaw - phi, math.tau)) < 1e-6

    def endpoint_commands():
        return [message for message in rig.commands[command_start:]
                if len(message.data) == 4 and
                abs(message.data[0] - expected_base) < 1e-6 and
                abs(message.data[3] - expected_wrist) < 1e-6]

    # Observe Joy's actual output after the follower has encoded the endpoint.
    rig.until(lambda: bool(endpoint_commands()))
    final_command = list(endpoint_commands()[0].data)
    assert all(math.isfinite(value) for value in final_command)
    if hold_equivalent_wrist:
        rig.until(lambda: rig.joints and
                  all(abs(a - b) < 1e-6
                      for a, b in zip(rig.joints[-1].data[:3], final_command[:3])))
        rig.observe(0.2)
        assert rig.joints[-1].data[3] == 0.0
        assert not result.done(), 'Equivalent orientation must not erase a physical wrist turn'
        rig.feedback_wrist = None

    wrapped_result = rig.resolve(result)
    assert wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped_result.result.success
    rig.until(lambda: joint_match(rig.joints[-1].data, final_command))
    rig.observe(0.1)
    assert joint_match(rig.commands[-1].data, final_command)
    assert joint_match(rig.joints[-1].data, final_command)
    # Compare raw joint angles; angular-distance wrapping would hide regressions.
    assert abs(rig.joints[-1].data[3] - expected_wrist) < 1e-6

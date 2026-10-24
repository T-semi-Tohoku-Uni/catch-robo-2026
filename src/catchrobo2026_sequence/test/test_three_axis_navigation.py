"""Verify three-axis lifecycle motion on isolated local ROS, without CAN."""

import math
import os
import signal
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from catchrobo2026_msgs.action import ExecuteSequence, FollowRoute
from catchrobo2026_msgs.srv import EndeffectorControl
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import Float32MultiArray
import pytest

from test_route_handoff import rig, waypoint_pose, xyz  # noqa: F401


def launch_stack(rig):
    rig.feedback = []
    rig.widths = []
    rig.subscriptions.append(rig.node.create_subscription(
        Float32MultiArray, 'current_joints', rig.feedback.append, 100))
    rig.width_service = rig.node.create_service(
        EndeffectorControl, 'set_endeffector_state',
        lambda request, response: record_width(rig, request, response))
    rig.robot = rig.launch('nav_director', 'dummy_robot_node',
                           {'initial_pose': '[670.0, -110.0, 220.0, 0.0]'})
    rig.launch('nav_director', 'path_generator_3d')
    rig.launch('nav_director', 'path_follower_node', remaps=('route:=follower_route',))
    rig.launch('catchrobo2026_hand_operated', 'joy_controller_node')
    assert rig.follow.wait_for_server(timeout_sec=15)
    assert rig.generate.wait_for_service(timeout_sec=15)
    rig.until(lambda: rig.node.count_subscribers('current_joints') >= 3)
    rig.observe(0.3)
    rig.until(lambda: len(rig.feedback) > 5 and len(rig.joint_commands) > 5)


def record_width(rig, request, response):
    rig.widths.append(request.command)
    response.success = True
    return response


def test_end_uses_xyz_three_joint_feedback_and_no_width(rig):
    launch_stack(rig)
    config = Path(get_package_share_directory('catchrobo2026_sequence')) / 'config/sequences.yaml'
    rig.launch('catchrobo2026_sequence', 'sequence_node', {'sequence_file': str(config)})
    assert rig.sequence.wait_for_server(timeout_sec=15)
    rig.execute(ExecuteSequence.Goal.START)
    rig.execute(ExecuteSequence.Goal.END)
    assert len(rig.routes) == 2
    assert math.dist(xyz(rig.routes[0].poses[-1]), (675, 400, 300)) < 1e-3
    assert math.dist(xyz(rig.routes[1].poses[-1]), (675, -110, 220)) < 1e-3
    assert all(len(message.data) == 3 for message in rig.feedback + rig.joint_commands)
    assert not rig.widths


@pytest.mark.parametrize('phi', [math.pi - 0.01, -math.pi + 0.01, 2 * math.pi])
def test_phi_wrap_cannot_restrict_xyz_arrival(rig, phi):
    launch_stack(rig)
    path = RosPath(poses=[PoseStamped(pose=waypoint_pose((675, 200, 200), phi))])
    handle = rig.resolve(rig.follow.send_goal_async(FollowRoute.Goal(start=True, path=path)))
    assert handle.accepted
    result = rig.resolve(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success
    assert all(len(message.data) == 3 for message in rig.feedback + rig.joint_commands)
    assert not rig.widths


def test_stale_three_axis_measurement_aborts_motion(rig):
    launch_stack(rig)
    os.killpg(rig.robot.pid, signal.SIGSTOP)
    rig.observe(1.15)
    path = RosPath(poses=[PoseStamped(pose=waypoint_pose((675, 400, 300), 0))])
    handle = rig.resolve(rig.follow.send_goal_async(FollowRoute.Goal(start=True, path=path)))
    assert handle.accepted
    result = rig.resolve(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED and not result.result.success
    assert not rig.widths


def test_base_zero_crossing_has_no_removed_wrist_wrap_constraint(rig):
    launch_stack(rig)
    path = RosPath(poses=[PoseStamped(pose=waypoint_pose(point, 0)) for point in
                         [(674, 200, 200), (676, 200, 200)]])
    handle = rig.resolve(rig.follow.send_goal_async(FollowRoute.Goal(start=True, path=path)))
    assert handle.accepted
    assert rig.resolve(handle.get_result_async()).result.success
    rig.observe(0.15)
    assert any(message.data[0] < 0 for message in rig.joint_commands)
    assert any(message.data[0] > 0 for message in rig.joint_commands)
    assert not rig.widths


@pytest.mark.parametrize('values', [[0.0, 0.0], [0.0] * 4, [0.0, float('nan'), 0.0]])
def test_invalid_feedback_rejected_before_follower_target(rig, values):
    launch_stack(rig)
    os.killpg(rig.robot.pid, signal.SIGSTOP)
    publisher = rig.node.create_publisher(Float32MultiArray, 'current_joints', 10)
    timer = rig.node.create_timer(0.02, lambda: publisher.publish(Float32MultiArray(data=values)))
    try:
        rig.observe(0.15)
        targets = len(rig.targets)
        path = RosPath(poses=[PoseStamped(pose=waypoint_pose((675, 400, 300), 0))])
        handle = rig.resolve(rig.follow.send_goal_async(FollowRoute.Goal(start=True, path=path)))
        assert handle.accepted
        result = rig.resolve(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_ABORTED and not result.result.success
        rig.observe(0.1)
        assert len(rig.targets) == targets
    finally:
        rig.node.destroy_timer(timer)


def test_cancel_holds_last_three_joint_command_without_width(rig):
    from sensor_msgs.msg import Joy
    launch_stack(rig)
    # Retired width button cannot produce a service request.
    publisher = rig.node.create_publisher(Joy, 'joy', 10)
    for buttons in [[0] * 6, [0, 0, 1, 0, 0, 0], [0] * 6]:
        publisher.publish(Joy(axes=[0.0] * 8, buttons=buttons))
        rig.observe(0.05)
    os.killpg(rig.robot.pid, signal.SIGSTOP)
    path = RosPath(poses=[PoseStamped(pose=waypoint_pose((675, 400, 300), 0))])
    handle = rig.resolve(rig.follow.send_goal_async(FollowRoute.Goal(start=True, path=path)))
    assert handle.accepted
    result = handle.get_result_async()
    rig.until(lambda: len(rig.targets) > 0)
    assert rig.resolve(handle.cancel_goal_async()).goals_canceling
    assert rig.resolve(result).status == GoalStatus.STATUS_CANCELED
    rig.observe(0.1)
    targets = len(rig.targets)
    command = list(rig.joint_commands[-1].data)
    rig.observe(0.15)
    assert len(rig.targets) == targets
    assert list(rig.joint_commands[-1].data) == command
    assert not rig.widths

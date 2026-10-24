"""Exercise the real target adapter against local RPC/feedback mocks."""
import struct
import time

from catchrobo2026_msgs.action import FollowTrajectory
from catchrobo2026_msgs.srv import ArmCommandLease, GenerateTrajectory
import pytest
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
import test_arm_target_client
from trajectory_msgs.msg import JointTrajectoryPoint

rig = test_arm_target_client.rig


def action(rig, duration=0.4):
    client = ActionClient(rig.node, FollowTrajectory, 'follow_trajectory')
    assert client.wait_for_server(timeout_sec=5)
    goal = FollowTrajectory.Goal()
    goal.trajectory.joint_names = ['theta1', 'theta2', 'theta3']
    start = list(rig.actual)
    end = [start[0] + 0.01, start[1], start[2]]
    goal.trajectory.points = [
        JointTrajectoryPoint(positions=start, velocities=[0.] * 3,
                             accelerations=[0.] * 3),
        JointTrajectoryPoint(positions=end, velocities=[0.] * 3,
                             accelerations=[0.] * 3)]
    goal.trajectory.points[1].time_from_start.sec = int(duration)
    goal.trajectory.points[1].time_from_start.nanosec = int(duration % 1 * 1e9)
    handle = rig.resolve(client.send_goal_async(goal))
    return client, handle


def lease(rig, operation=0, token=0):
    client = rig.node.create_client(ArmCommandLease, 'arm_target/route_lease')
    assert client.wait_for_service(timeout_sec=5)
    return rig.resolve(client.call_async(ArmCommandLease.Request(
        operation=operation, token=token,
        deadline_steady_ns=time.monotonic_ns() + 2_000_000_000)))


@pytest.mark.parametrize('rig', [{'trajectory': True}], indirect=True)
def test_action_success_rpc_and_post_ack_feedback(rig):
    rig.resume()
    rig.follow = True
    client, handle = action(rig)
    assert handle.accepted
    result = rig.resolve(handle.get_result_async())
    assert result.result.success, result.result.message
    assert rig.records and all(len(record[1]) == 12 for record in rig.records)
    assert not lease(rig, 1, 123).success
    assert lease(rig).success
    client.destroy()


@pytest.mark.parametrize('rig', [{'trajectory': True}], indirect=True)
def test_route_and_timed_are_exclusive_and_lease_expires(rig):
    rig.resume()
    held = lease(rig)
    assert held.success and held.token
    client, handle = action(rig)
    assert not handle.accepted
    assert not lease(rig).success
    rig.spin(0.6)
    assert not lease(rig, 1, held.token).success
    stop = rig.node.create_client(Trigger, 'arm_target/cancel_trajectory')
    assert stop.wait_for_service(timeout_sec=5)
    assert not rig.resolve(stop.call_async(Trigger.Request())).success
    rig.resume()
    assert lease(rig).success
    client.destroy()


@pytest.mark.parametrize('rig', [{'trajectory': True}], indirect=True)
def test_rpc_failure_aborts_action_and_latches_until_init(rig):
    rig.resume()
    rig.outcome = 2
    client, handle = action(rig)
    assert handle.accepted
    result = rig.resolve(handle.get_result_async())
    assert not result.result.success
    count = len(rig.records)
    rig.send([1., 2., 3.])
    rig.spin(0.1)
    assert len(rig.records) == count
    rig.outcome = 0
    rig.resume()
    rig.send([0.2, 0.3, 0.4])
    rig.until(lambda: len(rig.records) > count)
    client.destroy()


@pytest.mark.parametrize('rig', [{'trajectory': True}], indirect=True)
def test_stop_waits_for_inflight_then_rebase(rig):
    rig.resume()
    stop = rig.node.create_client(Trigger, 'arm_target/cancel_trajectory')
    assert stop.wait_for_service(timeout_sec=5)
    rig.spin(0.1)
    rig.hold = True
    client, handle = action(rig, 1.0)
    assert handle.accepted
    rig.until(lambda: bool(rig.records))
    stopped = stop.call_async(Trigger.Request())
    rig.spin(0.03)
    assert not stopped.done()
    rig.release()
    assert rig.resolve(stopped).success
    assert not rig.resolve(handle.get_result_async()).result.success
    count = len(rig.records)
    rig.spin(0.1)
    assert len(rig.records) == count
    client.destroy()


@pytest.mark.parametrize('rig', [{'trajectory': True}], indirect=True)
def test_init_preempts_owner_and_does_not_replay(rig):
    rig.resume()
    client, handle = action(rig, 1.0)
    assert handle.accepted
    rig.until(lambda: bool(rig.records))
    paused = rig.resolve(rig.gate())
    assert paused.success
    assert not rig.resolve(handle.get_result_async()).result.success
    count = len(rig.records)
    rig.spin(0.1)
    assert len(rig.records) == count
    rig.resume()
    rig.spin(0.1)
    assert len(rig.records) == count
    client.destroy()


def test_unconfigured_dynamics_rejects_only_timed(rig):
    rig.resume()
    client, handle = action(rig)
    assert not handle.accepted
    rig.send([0.2, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 1)
    assert rig.records[0][1] == struct.pack('<3f', 0.2, 0.3, 0.4)
    client.destroy()


@pytest.mark.parametrize('rig', [{'trajectory': True}], indirect=True)
def test_planning_service_returns_public_three_axes(rig):
    service = rig.node.create_client(GenerateTrajectory, 'generate_trajectory')
    assert service.wait_for_service(timeout_sec=5)
    request = GenerateTrajectory.Request()
    request.start.position = list(rig.actual)
    request.goal.position = [0.21, 0.3, 0.4]
    result = rig.resolve(service.call_async(request))
    assert result.success, result.message
    assert result.trajectory.joint_names == ['theta1', 'theta2', 'theta3']
    assert all(len(point.positions) == 3 for point in result.trajectory.points)


@pytest.mark.parametrize('rig', [{'trajectory': True}], indirect=True)
def test_final_ack_and_newer_feedback_are_both_required(rig):
    rig.resume()
    rig.follow = True
    rig.hold_final = True
    client, handle = action(rig, 0.2)
    assert handle.accepted
    result = handle.get_result_async()
    rig.until(lambda: bool(rig.waiters))
    rig.actual = [0.21, 0.3, 0.4]
    rig.spin(0.02)
    assert not result.done()
    rig.feedback_enabled = False
    rig.hold_final = False
    rig.release()
    rig.spin(0.08)
    assert not result.done()
    rig.feedback_enabled = True
    assert rig.resolve(result).result.success
    client.destroy()


@pytest.mark.parametrize('rig', [{'trajectory': True}], indirect=True)
def test_rebase_failure_latches_idle_stop_and_init_can_recover(rig):
    rig.resume()
    rig.follow = True
    rig.rebase_ok = False
    client, handle = action(rig, 0.2)
    assert handle.accepted
    assert not rig.resolve(handle.get_result_async()).result.success
    stop = rig.node.create_client(Trigger, 'arm_target/cancel_trajectory')
    assert stop.wait_for_service(timeout_sec=5)
    assert not rig.resolve(stop.call_async(Trigger.Request())).success
    rig.rebase_ok = True
    rig.resume()
    assert rig.resolve(stop.call_async(Trigger.Request())).success
    client.destroy()


@pytest.mark.parametrize('rig', [{'trajectory': True}], indirect=True)
def test_feedback_loss_aborts_without_fallback(rig):
    rig.resume()
    rig.feedback_enabled = False
    client, handle = action(rig, 2.0)
    assert handle.accepted
    assert not rig.resolve(handle.get_result_async()).result.success
    client.destroy()

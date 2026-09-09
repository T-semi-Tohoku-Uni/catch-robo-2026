"""Verify rotation-group preflight, route handoff and release without hardware."""

import copy

from action_msgs.msg import GoalStatus
from catchrobo2026_msgs.action import ExecuteSequence
from catchrobo2026_msgs.msg import RotationGroupRoute
from catchrobo2026_msgs.srv import PlanRotationGroup, PumpControl, WristControl
from geometry_msgs.msg import PoseStamped
import pytest
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

from test_waypoint_sequence import move, rig as base_rig, waypoint_step  # noqa: F401


@pytest.fixture
def rig(base_rig):  # noqa: F811
    rig = base_rig
    rig.group_plans, rig.wrist_requests = [], []
    rig.group_mode = 'success'
    rig.planned_phi_travel = 0.0
    rig.planned_interval_phi_travel = 0.0
    rig.begin_gate = None
    rig.pump_gate = None
    rig.wrist_mode = 'success'

    def plan(request, response):
        rig.group_plans.append(copy.deepcopy(request))
        response.success = rig.group_mode == 'success'
        response.message = 'no monotonic solution' if not response.success else 'planned'
        response.direction = 0 if request.allow_wrist_reversal else -1
        response.phi_travel = rig.planned_phi_travel
        response.interval_phi_travel = [rig.planned_interval_phi_travel] * len(
            request.phi_travel_intervals)
        if response.success:
            offset = 0
            previous = PoseStamped()
            previous.pose.position.x = 0.675
            previous.pose.position.y = 0.2
            previous.pose.position.z = 0.3
            previous.pose.orientation.w = 1.0
            wrist = -1.0
            for end in request.route_ends:
                route = RotationGroupRoute()
                route.path.header.frame_id = 'map'
                route.path.poses = [copy.deepcopy(previous)]
                route.wrist_angles = [wrist]
                for pose in request.targets[offset:end]:
                    previous = PoseStamped(pose=copy.deepcopy(pose))
                    route.path.poses.append(previous)
                    wrist -= 0.1
                    route.wrist_angles.append(wrist)
                if request.allow_wrist_reversal:
                    route.phi_angles = list(route.wrist_angles)
                response.routes.append(route)
                offset = end
        return response

    async def wrist(request, response):
        rig.wrist_requests.append(copy.deepcopy(request))
        if request.operation == WristControl.Request.BEGIN and rig.begin_gate is not None:
            await rig.begin_gate
        response.success = not (
            (rig.wrist_mode == 'reject_begin' and request.operation == WristControl.Request.BEGIN)
            or (rig.wrist_mode == 'reject_end' and request.operation == WristControl.Request.END))
        response.message = 'accepted' if response.success else 'rejected by test'
        return response

    async def pump(request, response):
        rig.pump(request, response)
        if rig.pump_gate is not None:
            await rig.pump_gate
        return response

    rig.node.destroy_service(rig.services[1])
    rig.services[1] = rig.node.create_service(
        PumpControl, 'set_pump_state', pump, callback_group=ReentrantCallbackGroup())
    rig.services.extend([
        rig.node.create_service(PlanRotationGroup, 'plan_rotation_group', plan),
        rig.node.create_service(WristControl, 'wrist_control', wrist,
                                callback_group=ReentrantCallbackGroup()),
    ])
    yield rig
    if rig.begin_gate is not None and not rig.begin_gate.done():
        rig.begin_gate.set_result(True)
    if rig.pump_gate is not None and not rig.pump_gate.done():
        rig.pump_gate.set_result(True)


def grouped(*steps):
    return [{'rotation_group': 'start'}, *steps, {'rotation_group': 'end'}]


def operations(rig):
    return [request.operation for request in rig.wrist_requests]


def assert_released(rig):
    assert operations(rig) == [WristControl.Request.BEGIN, WristControl.Request.END]
    assert rig.wrist_requests[0].group_id != 0
    assert rig.wrist_requests[0].group_id == rig.wrist_requests[1].group_id


def recover(rig):
    _, result = rig.start(ExecuteSequence.Goal.START)
    rig.until(lambda: len(rig.plans) == 1)
    rig.until(lambda: len(rig.follows) > 0 and
              rig.follows[-1]['request'].rotation_group_id == 0)
    rig.complete_follow(len(rig.follows) - 1)
    rig.assert_succeeded(result)
    assert not rig.follows[-1]['request'].wrist_angles


def test_plans_entire_group_and_preserves_waypoint_routes(rig):
    rig.launch(grouped(
        {'pump': 'suction'},
        waypoint_step([650, 200, 300, 0]),
        move([600, 200, 300, 0], waypoints=[[620, 200, 300, 0]]),
        {'wait': 0.03}, {'pump': 'off'},
        move([550, 200, 300, 0])), recovery_steps=[move([675, 200, 300, 0])])
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    assert len(rig.group_plans) == 1 and not rig.plans
    assert list(rig.group_plans[0].route_ends) == [3, 4]
    assert [pose.position.x for pose in rig.group_plans[0].targets] == pytest.approx(
        [0.650, 0.620, 0.600, 0.550])
    assert len(rig.pumps) == 1
    assert operations(rig) == [WristControl.Request.BEGIN]
    first = rig.follows[0]['request']
    assert first.rotation_group_id == rig.wrist_requests[0].group_id
    assert first.wrist_direction == -1
    assert len(first.path.poses) == len(first.wrist_angles) == 4
    rig.complete_follow(0)
    rig.until(lambda: len(rig.follows) == 2)
    second = rig.follows[1]['request']
    assert len(rig.pumps) == 2
    assert second.rotation_group_id == first.rotation_group_id
    assert second.wrist_angles[0] == first.wrist_angles[-1]
    rig.complete_follow(1)
    rig.assert_succeeded(result)
    assert_released(rig)
    recover(rig)


def test_impossible_group_fails_before_pump_or_begin(rig):
    rig.group_mode = 'reject'
    rig.launch(grouped({'pump': 'suction'}, move([600, 200, 300, 0])),
               recovery_steps=[move([675, 200, 300, 0])])
    _, result = rig.start()
    reply = rig.resolve(result)
    assert not reply.result.success and 'no monotonic solution' in reply.result.message
    assert not rig.pumps and not rig.follows and not rig.wrist_requests
    recover(rig)


@pytest.mark.parametrize('cancel', [False, True])
def test_route_failure_or_cancel_releases_before_recovery(rig, cancel):
    rig.launch(grouped(move([600, 200, 300, 0])),
               recovery_steps=[move([675, 200, 300, 0])])
    handle, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    if cancel:
        rig.resolve(handle.cancel_goal_async())
    else:
        rig.follows[0]['gate'].set_result(False)
    reply = rig.resolve(result)
    assert not reply.result.success
    assert reply.status == (GoalStatus.STATUS_CANCELED if cancel else GoalStatus.STATUS_ABORTED)
    assert_released(rig)
    recover(rig)


def test_cancel_waits_for_late_begin_and_releases(rig):
    rig.begin_gate = Future(executor=rig.executor)
    rig.launch(grouped({'pump': 'suction'}, move([600, 200, 300, 0])))
    handle, result = rig.start()
    rig.until(lambda: operations(rig) == [WristControl.Request.BEGIN])
    rig.resolve(handle.cancel_goal_async())
    rig.observe(0.1)
    assert not result.done() and not rig.follows and not rig.pumps
    rig.begin_gate.set_result(True)
    reply = rig.resolve(result)
    assert reply.status == GoalStatus.STATUS_CANCELED
    assert_released(rig)
    assert not rig.pumps and not rig.follows


def test_rejected_begin_does_not_send_commands(rig):
    rig.wrist_mode = 'reject_begin'
    rig.launch(grouped({'pump': 'suction'}, move([600, 200, 300, 0])))
    _, result = rig.start()
    assert not rig.resolve(result).result.success
    assert operations(rig) == [WristControl.Request.BEGIN]
    assert not rig.follows and not rig.pumps


def test_cancel_releases_while_pump_reply_is_pending(rig):
    rig.pump_gate = Future(executor=rig.executor)
    rig.launch(grouped({'pump': 'suction'}, move([600, 200, 300, 0])))
    handle, result = rig.start()
    rig.until(lambda: len(rig.pumps) == 1)
    rig.resolve(handle.cancel_goal_async())
    rig.until(lambda: len(rig.wrist_requests) == 2)
    assert_released(rig)
    assert not result.done() and not rig.follows
    rig.pump_gate.set_result(True)
    assert rig.resolve(result).status == GoalStatus.STATUS_CANCELED


def test_consecutive_groups_use_new_ids(rig):
    rig.launch(grouped(move([600, 200, 300, 0])) +
               grouped(move([550, 200, 300, 0])))
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    rig.complete_follow(0)
    rig.until(lambda: len(rig.follows) == 2)
    assert operations(rig) == [WristControl.Request.BEGIN, WristControl.Request.END,
                               WristControl.Request.BEGIN]
    assert rig.wrist_requests[2].group_id > rig.wrist_requests[0].group_id
    rig.complete_follow(1)
    rig.assert_succeeded(result)
    assert operations(rig)[-1] == WristControl.Request.END


def test_unconfirmed_release_blocks_next_action(rig):
    rig.wrist_mode = 'reject_end'
    rig.launch(grouped(move([600, 200, 300, 0])))
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    rig.complete_follow(0)
    reply = rig.resolve(result)
    assert not reply.result.success and 'stop unconfirmed' in reply.result.message
    handle = rig.resolve(rig.sequence.send_goal_async(ExecuteSequence.Goal(
        control_epoch=1, step_id=2, kind=ExecuteSequence.Goal.END, collector_mask=7)))
    assert not handle.accepted


@pytest.mark.parametrize('one_direction', [False, True])
def test_sequence_group_passes_independent_constraints(rig, one_direction):
    rig.launch([
        {'sequence_group': {'start': True, 'rotation_group': one_direction,
                            'max_phi_travel': 0.75}},
        move([600, 200, 300, 0]), {'wait': 0.05},
        move([650, 200, 300, 0]), {'sequence_group': 'end'},
    ])
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    request = rig.group_plans[0]
    assert request.allow_wrist_reversal == (not one_direction)
    assert request.limit_phi_travel and request.max_phi_travel == 0.75
    begin_request = rig.wrist_requests[0]
    assert begin_request.limit_phi_travel and begin_request.max_phi_travel == 0.75
    first = rig.follows[0]['request']
    assert first.wrist_direction == (-1 if one_direction else 0)
    assert bool(first.phi_angles) == (not one_direction)
    rig.complete_follow(0)
    rig.until(lambda: len(rig.follows) == 2)
    assert len(rig.wrist_requests) == 1
    assert rig.follows[1]['request'].rotation_group_id == first.rotation_group_id
    rig.complete_follow(1)
    rig.assert_succeeded(result)
    assert_released(rig)


def test_sequence_group_rejects_over_budget_plan_before_commands(rig):
    rig.planned_phi_travel = 1.0
    rig.launch([
        {'sequence_group': {'start': True, 'max_phi_travel': 0.5}},
        {'pump': 'suction'}, move([600, 200, 300, 0]), {'sequence_group': 'end'},
    ])
    _, result = rig.start()
    reply = rig.resolve(result)
    assert not reply.result.success and 'phi travel limit' in reply.result.message
    assert not rig.pumps and not rig.follows and not rig.wrist_requests


def test_phi_interval_flags_follow_arrival_and_preserve_group(rig):
    rig.launch([
        {'sequence_group': 'start'},
        {'move': {'absolute': [600, 200, 300, 0], 'waypoints': [[625, 210, 310, 0]]}},
        {'phi_travel': {'start': True, 'limit': 0.5235987755982988}}, {'wait': 0.05},
        move([650, 200, 300, 0]), {'phi_travel': 'end'},
        move([675, 200, 300, 0]), {'sequence_group': 'end'},
    ])
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    request = rig.group_plans[0]
    assert list(request.route_ends) == [2, 3, 4]
    assert len(request.phi_travel_intervals) == 1
    interval = request.phi_travel_intervals[0]
    assert (interval.start_target, interval.end_target) == (2, 3)
    assert operations(rig) == [WristControl.Request.BEGIN]
    assert not rig.wrist_requests[0].limit_phi_travel
    rig.complete_follow(0)
    rig.until(lambda: len(rig.follows) == 2)
    assert operations(rig) == [WristControl.Request.BEGIN, WristControl.Request.PHI_BEGIN]
    assert rig.wrist_requests[1].max_phi_travel == interval.max_phi_travel
    assert rig.wrist_requests[1].limit_phi_travel
    rig.complete_follow(1)
    rig.until(lambda: len(rig.follows) == 3)
    assert operations(rig)[-1] == WristControl.Request.PHI_END
    rig.complete_follow(2)
    rig.assert_succeeded(result)
    assert operations(rig) == [WristControl.Request.BEGIN, WristControl.Request.PHI_BEGIN,
                               WristControl.Request.PHI_END, WristControl.Request.END]
    assert len({request.group_id for request in rig.wrist_requests}) == 1


def test_phi_interval_over_budget_plan_is_rejected_before_approach(rig):
    rig.planned_interval_phi_travel = 1.0
    rig.launch([
        {'sequence_group': 'start'}, move([600, 200, 300, 0]),
        {'phi_travel': {'start': True, 'limit': 0.5}}, move([650, 200, 300, 0]),
        {'phi_travel': 'end'}, {'sequence_group': 'end'},
    ])
    _, result = rig.start()
    reply = rig.resolve(result)
    assert not reply.result.success and 'phi interval limit' in reply.result.message
    assert not rig.follows and not rig.wrist_requests


def test_cancel_inside_phi_interval_releases_outer_group(rig):
    rig.launch([
        {'sequence_group': 'start'}, move([600, 200, 300, 0]),
        {'phi_travel': {'start': True, 'limit': 0.5}}, move([650, 200, 300, 0]),
        {'phi_travel': 'end'}, {'sequence_group': 'end'},
    ])
    handle, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    rig.complete_follow(0)
    rig.until(lambda: len(rig.follows) == 2)
    rig.resolve(handle.cancel_goal_async())
    assert rig.resolve(result).status == GoalStatus.STATUS_CANCELED
    assert operations(rig) == [WristControl.Request.BEGIN, WristControl.Request.PHI_BEGIN,
                               WristControl.Request.END]

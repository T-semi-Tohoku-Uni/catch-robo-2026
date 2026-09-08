"""Verify deferred waypoints across UI actions using gated ROS mocks, without CAN."""

import math

from action_msgs.msg import GoalStatus
from catchrobo2026_msgs.action import ExecuteSequence
import pytest
from std_srvs.srv import Trigger
from test_rotation_group_sequence import base_rig  # noqa: F401
from test_rotation_group_sequence import rig as grouped_rig  # noqa: F401
from test_waypoint_sequence import move, waypoint_step
import yaml


@pytest.fixture
def rig(grouped_rig):  # noqa: F811
    rig = grouped_rig
    rig.initializations = []

    def initialize(request, response):
        rig.initializations.append(request)
        response.success = True
        return response

    rig.services.append(rig.node.create_service(Trigger, 'request_initialization', initialize))
    return rig


def launch(rig, place_steps, pick_steps, *, extra_sequences=None, debug=False):
    bindings = {
        'red': {'pick': {'0,1': 'next'}, 'place': {'0,0': 'tested'}},
        'blue': {'pick': {}, 'place': {}},
    }
    rig.launch(
        place_steps,
        sequences={**(extra_sequences or {}), 'next': {'steps': pick_steps}},
        config_options={'bindings': bindings, 'end_sequence': None},
        debug=debug)


def start(rig, kind=ExecuteSequence.Goal.PLACE, *, epoch=1, step_id=None):
    rig.step_id += 1
    request = ExecuteSequence.Goal(
        control_epoch=epoch, step_id=rig.step_id if step_id is None else step_id,
        kind=kind, collector_mask=7, row=0, column=1, box=0, box_column=0)
    handle = rig.resolve(rig.sequence.send_goal_async(request))
    assert handle.accepted
    return handle, handle.get_result_async()


def complete_next_route(rig, pending, count):
    rig.until(lambda: len(rig.follows) == count)
    rig.complete_follow(count - 1)
    rig.assert_succeeded(pending)


def points(request):
    return [(point.position.x * 1000, point.position.y * 1000,
             point.position.z * 1000) for point in request.waypoints]


def test_trailing_call_waits_for_next_move_after_leading_mechanisms(rig):
    launch(rig, [move([100, 200, 300, 0]), {'call': 'leave'}],
           [{'pump': 'off'}, {'endeffector': 1}, {'wait': 0.02},
            move([700, 250, 300, 0], waypoints=[[650, 200, 300, 0]])],
           extra_sequences={'leave': {'steps': [
               waypoint_step([400, 200, 300, 0]), waypoint_step([500, 200, 300, 0])]}})
    _, result = start(rig)
    complete_next_route(rig, result, 1)
    assert not rig.plans[0].waypoints
    rig.observe(0.1)
    assert len(rig.plans) == len(rig.follows) == 1
    _, result = start(rig, ExecuteSequence.Goal.PICK)
    rig.until(lambda: len(rig.follows) == 2)
    assert len(rig.pumps) == len(rig.endeffectors) == 1
    assert points(rig.plans[1]) == [(400, 200, 300), (500, 200, 300), (650, 200, 300)]
    rig.complete_follow(1)
    rig.assert_succeeded(result)


def test_waypoint_only_and_mechanism_only_actions_keep_pending_points(rig):
    launch(rig, [waypoint_step([400, 200, 300, 0])], [{'pump': 'off'}], debug=True)
    _, result = start(rig)
    rig.assert_succeeded(result)
    _, result = start(rig, ExecuteSequence.Goal.PICK)
    rig.assert_succeeded(result)
    _, result = start(rig)
    rig.assert_succeeded(result)
    assert not rig.plans and not rig.follows
    config = yaml.safe_load(rig.config_file.read_text())
    config['sequences']['next']['steps'] = [move([700, 250, 300, 0])]
    rig.config_file.write_text(yaml.safe_dump(config))
    _, result = start(rig, ExecuteSequence.Goal.PICK)
    complete_next_route(rig, result, 1)
    assert points(rig.plans[0]) == [(400, 200, 300), (400, 200, 300)]


@pytest.mark.parametrize('boundary', ['start', 'initialize', 'end', 'epoch', 'stale_id'])
def test_lifecycle_epoch_and_stale_identity_discard_pending(rig, boundary):
    launch(rig, [waypoint_step([400, 200, 300, 0])], [move([700, 250, 300, 0])])
    _, result = start(rig)
    rig.assert_succeeded(result)
    epoch, step_id = 1, None
    if boundary in ('start', 'initialize', 'end'):
        kind = {'start': ExecuteSequence.Goal.START,
                'initialize': ExecuteSequence.Goal.INITIALIZE,
                'end': ExecuteSequence.Goal.END}[boundary]
        _, result = start(rig, kind)
        rig.assert_succeeded(result)
        assert len(rig.initializations) == (1 if boundary == 'initialize' else 0)
        assert not rig.plans
    elif boundary == 'epoch':
        epoch = 2
    else:
        step_id = 1
    _, result = start(rig, ExecuteSequence.Goal.PICK, epoch=epoch, step_id=step_id)
    complete_next_route(rig, result, 1)
    assert not rig.plans[0].waypoints


@pytest.mark.parametrize('cancel', [False, True])
@pytest.mark.parametrize('already_pending', [False, True])
def test_failed_or_canceled_action_does_not_leave_waypoints(rig, cancel, already_pending):
    place = [waypoint_step([400, 200, 300, 0])]
    if not already_pending:
        place.insert(0, move([100, 200, 300, 0]))
    launch(rig, place, [move([700, 250, 300, 0])])
    if already_pending:
        _, result = start(rig)
        rig.assert_succeeded(result)
        handle, result = start(rig, ExecuteSequence.Goal.PICK)
    else:
        handle, result = start(rig)
    rig.until(lambda: len(rig.follows) == 1)
    assert len(rig.plans[0].waypoints) == (1 if already_pending else 0)
    if cancel:
        rig.resolve(handle.cancel_goal_async())
    else:
        rig.follows[0]['gate'].set_result(False)
    reply = rig.resolve(result)
    assert not reply.result.success
    assert reply.status == (GoalStatus.STATUS_CANCELED if cancel else GoalStatus.STATUS_ABORTED)
    _, result = start(rig, ExecuteSequence.Goal.PICK)
    complete_next_route(rig, result, 2)
    assert not rig.plans[1].waypoints


def test_configuration_failure_discards_pending(rig):
    launch(rig, [waypoint_step([400, 200, 300, 0])], [move([700, 250, 300, 0])], debug=True)
    _, result = start(rig)
    rig.assert_succeeded(result)
    original = rig.config_file.read_text()
    rig.config_file.write_text('version: [invalid]\n')
    _, result = start(rig, ExecuteSequence.Goal.PICK)
    reply = rig.resolve(result)
    assert not reply.result.success and 'configuration:' in reply.result.message
    assert not rig.plans
    rig.config_file.write_text(original)
    _, result = start(rig, ExecuteSequence.Goal.PICK)
    complete_next_route(rig, result, 1)
    assert not rig.plans[0].waypoints


def test_relative_tail_keeps_original_anchor_across_debug_reload(rig):
    launch(rig, [move([100, 200, 300, 0]), waypoint_step([10, 20, 30, 0.1], relative=True)],
           [move([700, 250, 300, 0])], debug=True)
    _, result = start(rig)
    complete_next_route(rig, result, 1)
    config = yaml.safe_load(rig.config_file.read_text())
    config['sequences']['tested']['steps'][0] = move([900, 900, 900, 0])
    rig.config_file.write_text(yaml.safe_dump(config))
    _, result = start(rig, ExecuteSequence.Goal.PICK)
    complete_next_route(rig, result, 2)
    assert points(rig.plans[1]) == [(110, 220, 330)]
    q = rig.plans[1].waypoints[0].orientation
    assert 2 * math.atan2(q.z, q.w) == pytest.approx(0.1)


@pytest.mark.parametrize('reject', [False, True])
def test_pending_waypoint_is_planned_inside_the_next_group(rig, reject):
    group = [{'sequence_group': {'start': True, 'max_phi_travel': 0.5}},
             {'pump': 'off'}, move([700, 250, 300, 0]), {'sequence_group': 'end'}]
    launch(rig, [waypoint_step([400, 200, 300, 0])], group)
    _, result = start(rig)
    rig.assert_succeeded(result)
    rig.group_mode = 'reject' if reject else 'success'
    _, result = start(rig, ExecuteSequence.Goal.PICK)
    rig.until(lambda: len(rig.group_plans) == 1)
    request = rig.group_plans[0]
    assert request.limit_phi_travel and request.max_phi_travel == 0.5
    assert [point.position.x for point in request.targets] == pytest.approx([0.4, 0.7])
    assert list(request.route_ends) == [2]
    if reject:
        assert not rig.resolve(result).result.success
        assert not rig.pumps and not rig.follows and not rig.wrist_requests
    else:
        complete_next_route(rig, result, 1)
        assert len(rig.pumps) == 1 and len(rig.wrist_requests) == 2
    assert not rig.plans

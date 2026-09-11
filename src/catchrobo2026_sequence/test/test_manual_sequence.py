"""Verify manual handoff and resumption against gated ROS operations."""

from pathlib import Path

from action_msgs.msg import GoalStatus
from catchrobo2026_msgs.action import ExecuteSequence
from catchrobo2026_msgs.srv import SetSequenceManual, WristControl
import pytest
from rclpy.task import Future

from test_rotation_group_sequence import grouped, operations, rig as group_rig  # noqa: F401
from test_suction_sequence import CheckReply, rig as suction_rig  # noqa: F401
from test_waypoint_sequence import move, rig as base_rig  # noqa: F401


@pytest.fixture
def rig(group_rig):  # noqa: F811
    group_rig.manual = group_rig.node.create_client(SetSequenceManual, 'set_sequence_manual')
    yield group_rig
    group_rig.node.destroy_client(group_rig.manual)


def set_manual(rig, manual, *, epoch=1, step_id=None, token=None):
    assert rig.manual.wait_for_service(timeout_sec=5)
    if token is None:
        token = rig.feedback[-1].manual_token if rig.feedback else 0
    return rig.resolve(rig.manual.call_async(SetSequenceManual.Request(
        control_epoch=epoch, step_id=rig.step_id if step_id is None else step_id,
        manual=manual, manual_token=token)))


def waiting(rig):
    return bool(rig.feedback and rig.feedback[-1].phase.startswith('manual waiting'))


def wait_manual(rig):
    rig.until(lambda: waiting(rig))


def test_yaml_manual_stops_before_next_move_and_rejects_stale_control(rig):
    rig.launch([move([600, 200, 300, 0]), {'manual': {'label': 'Adjust pickup'}},
                move([550, 200, 300, 0])])
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    rig.complete_follow(0)
    wait_manual(rig)
    assert rig.feedback[-1].phase == 'manual waiting: Adjust pickup'
    assert rig.feedback[-1].step_index == 1
    assert rig.feedback[-1].manual_token == 1
    rig.observe(0.15)
    assert len(rig.plans) == 1 and not result.done()
    assert not set_manual(rig, False, epoch=2).success
    assert not set_manual(rig, False, step_id=rig.step_id + 1).success
    assert waiting(rig)
    assert set_manual(rig, True).success
    assert set_manual(rig, False).success
    rig.until(lambda: len(rig.follows) == 2)
    assert len(rig.plans) == 2
    rig.complete_follow(1)
    rig.assert_succeeded(result)
    assert not set_manual(rig, False).success


def test_source_pick_waits_at_manual_gate_before_retreat(rig):
    config_file = Path(__file__).parents[1] / 'config/sequences.yaml'
    rig.launch_file(config_file)
    _, result = rig.start(ExecuteSequence.Goal.PICK, row=0, column=1)

    for index in range(3):
        rig.until(lambda: len(rig.follows) == index + 1)
        rig.complete_follow(index)

    wait_manual(rig)
    assert rig.feedback[-1].step_index == 4
    assert rig.feedback[-1].manual_token == 1
    assert len(rig.plans) == len(rig.follows) == 3
    assert len(rig.pumps) == 1 and not rig.endeffectors
    rig.observe(0.4)
    assert len(rig.plans) == len(rig.follows) == 3 and not result.done()

    assert set_manual(rig, False).success
    rig.until(lambda: len(rig.follows) == 4)
    assert len(rig.plans) == 4
    rig.complete_follow(3)
    rig.assert_succeeded(result)
    assert len(rig.endeffectors) == 1


def test_ui_manual_waits_for_move_completion_without_canceling_it(rig):
    rig.launch([move([600, 200, 300, 0]), {'pump': 'suction'},
                move([550, 200, 300, 0])])
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    assert set_manual(rig, True).success
    rig.until(lambda: rig.feedback[-1].phase.startswith('manual requested'))
    rig.observe(0.15)
    assert not waiting(rig) and not rig.pumps and rig.cancel_count == 0
    rig.complete_follow(0)
    wait_manual(rig)
    assert not rig.pumps and len(rig.follows) == 1
    assert rig.feedback[-1].manual_token == 1
    assert not set_manual(rig, False, token=0).success
    assert waiting(rig) and not rig.pumps
    assert set_manual(rig, False).success
    rig.until(lambda: len(rig.follows) == 2)
    assert len(rig.pumps) == 1
    rig.complete_follow(1)
    rig.assert_succeeded(result)


def test_manual_during_group_waits_for_all_routes_and_end_acknowledgment(rig):
    rig.end_gate = Future(executor=rig.executor)
    rig.launch(grouped(move([600, 200, 300, 0]), move([550, 200, 300, 0])) +
               [{'pump': 'suction'}])
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    assert set_manual(rig, True).success
    rig.complete_follow(0)
    rig.until(lambda: len(rig.follows) == 2)
    assert not waiting(rig)
    rig.complete_follow(1)
    rig.until(lambda: operations(rig)[-1] == WristControl.Request.END)
    rig.observe(0.15)
    assert not waiting(rig) and not rig.pumps
    rig.end_gate.set_result(True)
    wait_manual(rig)
    assert not rig.pumps and not result.done()
    assert rig.cancel_count == 0
    assert set_manual(rig, False).success
    rig.assert_succeeded(result)
    assert len(rig.pumps) == 1


def test_manual_wait_freezes_remaining_wait_and_sequence_timeout(rig):
    rig.launch([{'wait': 1.2}, {'pump': 'off'}], sequence_timeout_sec=1.7)
    _, result = rig.start()
    rig.until(lambda: rig.feedback and rig.feedback[-1].phase == 'waiting')
    rig.observe(0.15)
    assert set_manual(rig, True).success
    wait_manual(rig)
    rig.observe(1.9)
    assert not result.done() and not rig.pumps
    assert set_manual(rig, False).success
    rig.observe(0.35)
    assert not result.done() and not rig.pumps
    rig.assert_succeeded(result)
    assert len(rig.pumps) == 1


def test_manual_waits_for_active_service_response(rig):
    rig.pump_gate = Future(executor=rig.executor)
    rig.launch([{'pump': 'suction'}, {'endeffector': 1}])
    _, result = rig.start()
    rig.until(lambda: len(rig.pumps) == 1)
    assert set_manual(rig, True).success
    rig.observe(0.15)
    assert not waiting(rig) and not rig.endeffectors
    rig.pump_gate.set_result(True)
    wait_manual(rig)
    assert not rig.endeffectors
    assert set_manual(rig, False).success
    rig.assert_succeeded(result)
    assert len(rig.endeffectors) == 1


def test_pending_request_can_be_withdrawn_without_later_pause(rig):
    rig.launch([move([600, 200, 300, 0]), {'pump': 'suction'}])
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    assert set_manual(rig, True).success
    assert rig.feedback[-1].manual_token == 0
    assert set_manual(rig, False).success
    rig.complete_follow(0)
    rig.assert_succeeded(result)
    assert len(rig.pumps) == 1
    assert not any(feedback.phase.startswith('manual waiting') for feedback in rig.feedback)


def test_cancel_while_manual_does_not_complete_or_execute_following_steps(rig):
    rig.launch([{'manual': True}, {'pump': 'suction'}], recovery_steps=[{'wait': 0}])
    goal, result = rig.start()
    wait_manual(rig)
    rig.resolve(goal.cancel_goal_async())
    reply = rig.resolve(result)
    assert reply.status == GoalStatus.STATUS_CANCELED and not reply.result.success
    assert not rig.pumps
    _, next_result = rig.start(ExecuteSequence.Goal.START)
    rig.assert_succeeded(next_result)


def test_failed_move_cannot_become_manual_or_successful(rig):
    rig.launch([move([600, 200, 300, 0]), {'pump': 'suction'}])
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    assert set_manual(rig, True).success
    rig.follows[0]['gate'].set_result(False)
    reply = rig.resolve(result)
    assert reply.status == GoalStatus.STATUS_ABORTED and not reply.result.success
    assert not rig.pumps
    assert not any(feedback.phase.startswith('manual waiting') for feedback in rig.feedback)


def test_rejected_group_release_never_grants_manual_control(rig):
    rig.wrist_mode = 'reject_end'
    rig.launch(grouped(move([600, 200, 300, 0])) + [{'pump': 'suction'}])
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    assert set_manual(rig, True).success
    rig.complete_follow(0)
    reply = rig.resolve(result)
    assert reply.status == GoalStatus.STATUS_ABORTED and not reply.result.success
    assert 'release rejected' in reply.result.message
    assert not rig.pumps
    assert not any(feedback.phase.startswith('manual waiting') for feedback in rig.feedback)


def test_ui_manual_waits_for_suction_check_completion(suction_rig):
    rig = suction_rig
    rig.feedback = []
    rig.manual = rig.node.create_client(SetSequenceManual, 'set_sequence_manual')
    rig.check_replies.append(CheckReply(True, hold=True))
    rig.launch([{'suction_check': {'start': {'timeout': 1.0}}},
                {'suction_check': 'wait'}, {'pump': 'suction'}])
    rig.step_id = 1
    goal = rig.resolve(rig.sequence.send_goal_async(ExecuteSequence.Goal(
        control_epoch=1, step_id=rig.step_id, kind=ExecuteSequence.Goal.PICK,
        row=0, column=1, collector_mask=7),
        feedback_callback=lambda message: rig.feedback.append(message.feedback)))
    assert goal.accepted
    result = goal.get_result_async()
    rig.until(lambda: 0 in rig.held)
    assert set_manual(rig, True).success
    rig.observe(0.15)
    assert not waiting(rig) and not rig.pumps
    rig.release(0)
    wait_manual(rig)
    assert not rig.pumps
    assert set_manual(rig, False).success
    reply = rig.resolve(result)
    assert reply.status == GoalStatus.STATUS_SUCCEEDED and reply.result.success
    assert rig.pumps == [(1, 1, 1)]
    rig.node.destroy_client(rig.manual)


def test_delayed_resume_cannot_release_the_next_manual_wait_in_one_pick(rig):
    rig.launch([{'manual': True}, {'pump': 'off'}, {'manual': True}, {'pump': 'suction'}],
               config_options={'bindings': {
                   'red': {'pick': {'0,1': 'tested'}, 'place': {}},
                   'blue': {'pick': {}, 'place': {}},
               }})
    rig.step_id = 1
    goal = rig.resolve(rig.sequence.send_goal_async(ExecuteSequence.Goal(
        control_epoch=1, step_id=rig.step_id, kind=ExecuteSequence.Goal.PICK,
        row=0, column=1, collector_mask=7),
        feedback_callback=lambda message: rig.feedback.append(message.feedback)))
    assert goal.accepted
    result = goal.get_result_async()
    wait_manual(rig)
    first_token = rig.feedback[-1].manual_token
    assert first_token == 1
    delayed_resume = SetSequenceManual.Request(
        control_epoch=1, step_id=rig.step_id, manual=False, manual_token=first_token)
    assert set_manual(rig, False, token=first_token).success
    rig.until(lambda: waiting(rig) and rig.feedback[-1].manual_token == 2)
    assert len(rig.pumps) == 1 and rig.pumps[0].left == 0
    rejected = rig.resolve(rig.manual.call_async(delayed_resume))
    assert not rejected.success and 'token' in rejected.message
    rig.observe(0.15)
    assert waiting(rig) and len(rig.pumps) == 1 and not result.done()
    assert set_manual(rig, False, token=2).success
    rig.assert_succeeded(result)
    assert len(rig.pumps) == 2 and rig.pumps[1].left == 1


def test_manual_token_changes_for_ui_waits_and_resets_in_the_next_action(rig):
    rig.launch([move([600, 200, 300, 0]), move([550, 200, 300, 0])],
               recovery_steps=[move([500, 200, 300, 0])])
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    assert set_manual(rig, True, token=99).success
    rig.complete_follow(0)
    wait_manual(rig)
    assert rig.feedback[-1].manual_token == 1
    assert set_manual(rig, False).success
    rig.until(lambda: len(rig.follows) == 2)
    assert set_manual(rig, True).success
    rig.complete_follow(1)
    rig.until(lambda: waiting(rig) and rig.feedback[-1].manual_token == 2)
    assert not set_manual(rig, False, token=1).success
    assert set_manual(rig, False).success
    rig.assert_succeeded(result)
    _, next_result = rig.start(ExecuteSequence.Goal.START)
    rig.until(lambda: len(rig.follows) == 3)
    rig.until(lambda: rig.feedback[-1].manual_token == 0)
    assert set_manual(rig, True).success
    assert not set_manual(rig, False, token=2).success
    assert set_manual(rig, False, token=0).success
    rig.complete_follow(2)
    rig.assert_succeeded(next_result)


def test_manual_wait_feedback_repeats_without_changing_token_or_advancing(rig):
    rig.launch([{'manual': {'label': 'Adjust pickup'}}, move([600, 200, 300, 0])])
    _, result = rig.start()
    wait_manual(rig)
    rig.feedback.clear()
    rig.until(lambda: len(rig.feedback) >= 3)
    assert all(feedback.phase == 'manual waiting: Adjust pickup' and
               feedback.manual_token == 1 and feedback.step_index == 0
               for feedback in rig.feedback)
    assert not rig.plans and not result.done()
    assert set_manual(rig, False).success
    rig.until(lambda: len(rig.follows) == 1)
    rig.until(lambda: rig.feedback[-1].phase == 'following route')
    rig.feedback.clear()
    rig.observe(0.5)
    assert not rig.feedback
    rig.complete_follow(0)
    rig.assert_succeeded(result)


def test_pending_manual_feedback_repeats_while_the_active_move_continues(rig):
    rig.launch([move([600, 200, 300, 0]), {'pump': 'suction'}])
    _, result = rig.start()
    rig.until(lambda: len(rig.follows) == 1)
    rig.until(lambda: rig.feedback[-1].phase == 'following route')
    assert set_manual(rig, True).success
    rig.until(lambda: rig.feedback[-1].phase == 'manual requested: following route')
    rig.feedback.clear()
    rig.until(lambda: len(rig.feedback) >= 3)
    assert all(feedback.phase == 'manual requested: following route' and
               feedback.manual_token == 0 and feedback.step_index == 0
               for feedback in rig.feedback)
    assert rig.cancel_count == 0 and not rig.pumps and not result.done()
    assert set_manual(rig, False, token=0).success
    rig.complete_follow(0)
    rig.assert_succeeded(result)

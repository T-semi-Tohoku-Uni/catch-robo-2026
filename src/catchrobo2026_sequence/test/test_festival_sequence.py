"""Verify festival lifecycle commands using local ROS mocks only."""

from action_msgs.msg import GoalStatus
from catchrobo2026_msgs.action import ExecuteSequence
from std_msgs.msg import Float32MultiArray
import pytest

from festival_support import CheckReply, rig  # noqa: F401


def feedback(rig, phi=-1.2):
    publisher = rig.node.create_publisher(Float32MultiArray, 'current_joints', 10)
    timer = rig.node.create_timer(0.02, lambda: publisher.publish(
        Float32MultiArray(data=[0.3, 0.4, 0.5, phi - 0.3])))
    rig.timers.append(timer)
    rig.observe(0.2)
    return timer


@pytest.mark.parametrize('kind', [ExecuteSequence.Goal.PICK, ExecuteSequence.Goal.PLACE, 3, 255])
def test_removed_game_goals_are_rejected(rig, kind):
    rig.launch([{'pump': 'suction'}])
    handle = rig.resolve(rig.sequence.send_goal_async(ExecuteSequence.Goal(
        control_epoch=1, step_id=1, kind=kind, collector_mask=7, row=0, column=1)))
    assert not handle.accepted
    assert not rig.pumps and not rig.routes and not rig.widths


@pytest.mark.parametrize('selected', [1, 2, 3])
def test_lifecycle_pump_only_addresses_configured_collector(rig, selected):
    rig.launch([{'pump': 'suction'}, {'pump': 'release'}, {'pump': 'off'}],
               selected_pump=selected)
    _, result = rig.start(mask=7)
    assert rig.resolve(result).result.success
    expected = []
    for state in [1, -1, 0]:
        output = [0, 0, 0]
        output[selected - 1] = state
        expected.append(tuple(output))
    assert rig.pumps == expected


def test_moves_and_waypoints_hold_starting_phi(rig):
    rig.launch([
        {'move': {'absolute': [600, 200, 300, 2.0],
                  'waypoints': [[610, 200, 310, -3.0]]}},
        {'move': {'absolute': [670, -110, 220, 0.0]}},
    ])
    feedback(rig)
    _, result = rig.start()
    assert rig.resolve(result).result.success
    assert len(rig.group_plans) == 2
    import math
    for plan in rig.group_plans:
        assert plan.allow_wrist_reversal and plan.limit_phi_travel
        assert plan.max_phi_travel == 0.0
        for target in plan.targets:
            assert abs(2 * math.atan2(target.orientation.z, target.orientation.w) + 1.2) < 1e-6
    assert len(rig.group_plans[0].targets) == 2
    assert len(rig.follow_goals) == 2
    assert all(goal.rotation_group_id != 0 for goal in rig.follow_goals)
    assert len(rig.wrist_operations) == 4



@pytest.mark.parametrize('invalid', [False, True])
def test_missing_or_invalid_feedback_never_starts_a_move(rig, invalid):
    rig.launch([{'endeffector': 1}, {'move': {'absolute': [670, -110, 220, 0]}}])
    if invalid:
        publisher = rig.node.create_publisher(Float32MultiArray, 'current_joints', 10)
        timer = rig.node.create_timer(0.02, lambda: publisher.publish(
            Float32MultiArray(data=[0.0, 0.0, 0.0, float('nan')])))
        rig.timers.append(timer)
        rig.observe(0.2)
    _, result = rig.start()
    outcome = rig.resolve(result)
    assert not outcome.result.success
    assert 'current_joints' in outcome.result.message
    assert not rig.routes and not rig.widths


def test_stale_feedback_never_starts_a_move(rig):
    rig.launch([{'move': {'absolute': [670, -110, 220, 0]}}])
    timer = feedback(rig)
    timer.cancel()
    rig.observe(1.1)
    _, result = rig.start()
    assert not rig.resolve(result).result.success
    assert not rig.routes


def test_cancel_wait_stops_before_following_pump_and_allows_restart(rig):
    rig.launch([{'wait': 1.0}, {'pump': 'suction'}])
    handle, result = rig.start()
    rig.observe(0.05)
    assert rig.resolve(handle.cancel_goal_async()).goals_canceling
    outcome = rig.resolve(result)
    assert outcome.status == GoalStatus.STATUS_CANCELED
    assert not outcome.result.success and not rig.pumps
    _, restarted = rig.start()
    assert rig.resolve(restarted).result.success
    assert rig.pumps == [(1, 0, 0)]


def test_rotation_group_is_rejected_before_width_or_motion(rig):
    rig.launch([{'endeffector': 1}, {'sequence_group': 'start'},
                {'move': {'absolute': [670, -110, 220, 0]}}, {'sequence_group': 'end'}])
    feedback(rig)
    _, result = rig.start()
    assert not rig.resolve(result).result.success
    assert not rig.widths and not rig.routes and not rig.group_plans


@pytest.mark.parametrize('selected', [1, 2, 3])
def test_suction_check_uses_only_selected_collector(rig, selected):
    rig.launch([{'suction_check': {'start': {'timeout': 0.5}}},
                {'suction_check': 'wait'}], selected_pump=selected)
    rig.check_replies.append(CheckReply(success=True))
    _, result = rig.start(mask=7)
    assert rig.resolve(result).result.success
    assert rig.checks == [(1 << (selected - 1), 0.5)]


def test_manual_wait_is_rejected_before_any_operation(rig):
    rig.launch([{'endeffector': 1}, {'manual': True}, {'pump': 'suction'}])
    _, result = rig.start()
    outcome = rig.resolve(result)
    assert not outcome.result.success
    assert 'manual sequence waits are disabled' in outcome.result.message
    assert not rig.widths and not rig.pumps and not rig.routes


@pytest.mark.parametrize('success', [False, True])
def test_internal_phi_groups_preserve_branch_targets_and_terminal_jump(rig, success):
    rig.check_replies.append(CheckReply(success=success, timed_out=not success))
    rig.launch([
        {'suction_check': {'start': {'timeout': 0.5}}}, {'suction_check': 'wait'},
        {'if': {'condition': 'suction_success',
                'then': [{'move': {'absolute': [600, 200, 300, 2.0]}}],
                'else': [{'move': {'absolute': [610, 200, 300, -2.0]}}]}},
        {'move': {'absolute': [670, -110, 220, 0.0]}},
    ])
    feedback(rig)
    _, result = rig.start()
    assert rig.resolve(result).result.success
    assert len(rig.group_plans) == 2
    expected_x = 0.6 if success else 0.61
    assert abs(rig.group_plans[0].targets[-1].position.x - expected_x) < 1e-8
    assert abs(rig.group_plans[1].targets[-1].position.x - 0.67) < 1e-8


@pytest.mark.parametrize('success', [False, True])
def test_internal_phi_groups_preserve_jump_to_sequence_end(rig, success):
    rig.check_replies.append(CheckReply(success=success, timed_out=not success))
    rig.launch([
        {'suction_check': {'start': {'timeout': 0.5}}}, {'suction_check': 'wait'},
        {'if': {'condition': 'suction_success',
                'then': [{'move': {'absolute': [600, 200, 300, 0.0]}}],
                'else': [{'move': {'absolute': [610, 200, 300, 0.0]}}]}},
    ])
    feedback(rig)
    _, result = rig.start()
    assert rig.resolve(result).result.success
    assert len(rig.group_plans) == 1


def test_cancel_during_internal_phi_route_releases_wrist_before_terminal(rig):
    rig.hold_follow = True
    rig.launch([{'move': {'absolute': [600, 200, 300, 0.0]}}, {'pump': 'suction'}])
    feedback(rig)
    handle, result = rig.start()
    rig.until(lambda: rig.follow_waiter is not None)
    assert rig.resolve(handle.cancel_goal_async()).goals_canceling
    rig.until(lambda: len(rig.wrist_operations) == 2)
    assert not result.done() and not rig.pumps
    rig.follow_waiter.set_result(True)
    outcome = rig.resolve(result)
    assert outcome.status == GoalStatus.STATUS_CANCELED and not outcome.result.success
    assert not rig.pumps

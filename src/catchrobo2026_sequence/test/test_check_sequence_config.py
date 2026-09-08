"""Validate the offline CLI contract without joining a ROS graph."""

import json
import math
import os
from pathlib import Path
import subprocess

import pytest


@pytest.fixture(scope='module')
def checker():
    configured = os.environ.get('CHECK_SEQUENCE_CONFIG_EXECUTABLE')
    if configured:
        executable = Path(configured)
    else:
        from ament_index_python.packages import get_package_prefix
        executable = Path(get_package_prefix('catchrobo2026_sequence')) / (
            'lib/catchrobo2026_sequence/check_sequence_config')
    assert executable.is_file(), f'Offline checker was not built: {executable}'
    return executable


@pytest.fixture
def config_file(tmp_path):
    path = tmp_path / 'offline "checker".yaml'
    path.write_text("""version: 1
poses:
  A: [675, 200, 300, 0]
  B: [670, -110, 220, 0]
sequences:
  ending:
    steps:
      - sequence_group: {start: true, max_phi_travel: 0}
      - move: {absolute: B}
      - sequence_group: end
  shift:
    steps:
      - move: {absolute: [700, 200, 300, 0]}
  unreachable:
    steps:
      - move: {absolute: [10000000, 200, 300, 0]}
end_sequence: ending
bindings:
  red:
    pick: {'0,1': shift}
    place: {}
  blue:
    pick: {'0,1': shift}
    place: {}
""")
    return path


@pytest.fixture
def continuation_file(config_file):
    additions = """  defer:
    steps:
      - waypoint:
          absolute: [720, 200, 300, 0]
  defer_again:
    steps:
      - move: {absolute: [750, 200, 300, 0]}
      - waypoint: {absolute: [760, 200, 300, 0]}
  mechanism_only:
    steps:
      - endeffector: 0
  finish:
    steps:
      - move: {absolute: [780, 200, 300, 0]}
  far_waypoint:
    steps:
      - waypoint: {absolute: [5000, 200, 300, 0]}
  relative_tail:
    steps:
      - move: {absolute: [720, 200, 300, 0]}
      - waypoint: {relative: [0, 20, 0, 0]}
"""
    contents = config_file.read_text().replace(
        'end_sequence: ending', additions + 'end_sequence: ending')
    contents = contents.replace('place: {}', "place: {'0,0': defer}")
    config_file.write_text(contents)
    return config_file


@pytest.fixture
def suction_file(continuation_file):
    additions = """  inspect_suction:
    steps:
      - suction_check: {start: {timeout: 0.1}}
      - suction_check: wait
      - if:
          condition: suction_success
          then: [{pump: suction}]
          else: [{fail: 'No suction'}]
  inspect_after_move:
    steps:
      - move: {absolute: [750, 200, 300, 0]}
      - call: inspect_suction
      - waypoint: {absolute: [760, 200, 300, 0]}
  break_before_failure:
    steps:
      - for:
          max_iterations: 3
          steps:
            - break: true
            - fail: 'Unreachable failure'
      - move: {absolute: [750, 200, 300, 0]}
  intentional_failure:
    steps:
      - fail: 'Explicit failure'
      - move: {absolute: [750, 200, 300, 0]}
"""
    continuation_file.write_text(continuation_file.read_text().replace(
        'end_sequence: ending', additions + 'end_sequence: ending'))
    return continuation_file


def run_check(checker, config_file, *args):
    result = subprocess.run(
        [str(checker), '--config', str(config_file), '--json', *map(str, args)],
        capture_output=True, text=True, timeout=30,
    )
    assert not result.stderr, result.stderr
    payload = json.loads(result.stdout)
    assert payload['exit_code'] == result.returncode
    return result.returncode, payload


def starting_at_a(wrist):
    return ['--initial-pose-name', 'A', '--initial-wrist', str(wrist)]


def test_syntax_only_and_invalid_configuration(checker, config_file):
    code, report = run_check(checker, config_file, '--syntax-only')
    assert code == 0
    assert report['mode'] == 'syntax'
    assert report['status'] == 'FEASIBLE'
    assert report['initial_joints'] is None
    assert report['results'] == []
    assert report['config'] == str(config_file)
    config_file.write_text(config_file.read_text().replace(
        'max_phi_travel: 0', 'max_phi_travel: -1'))
    code, report = run_check(checker, config_file, '--syntax-only')
    assert code == 1
    assert report['status'] == 'INFEASIBLE'
    assert 'max_phi_travel' in report['initial_message']


def test_missing_or_ambiguous_initial_state_is_unknown(checker, config_file):
    for initial in [[], ['--initial-pose-name', 'A']]:
        code, report = run_check(
            checker, config_file, '--team', 'red', '--action', 'end', *initial)
        assert code == 2
        assert report['status'] == 'UNKNOWN'
        assert report['initial_joints'] is None
        assert report['results'][0]['status'] == 'UNKNOWN'
        assert report['results'][0]['final_joints'] is None
    assert '--initial-wrist' in report['initial_message']


@pytest.mark.parametrize('wrist,expected', [(0, 'INFEASIBLE'), (-2 * math.pi, 'FEASIBLE')])
def test_ending_distinguishes_initial_wrist_branches(checker, config_file, wrist, expected):
    code, report = run_check(
        checker, config_file, '--team', 'both', '--action', 'end', *starting_at_a(wrist))
    assert code == (0 if expected == 'FEASIBLE' else 1)
    assert report['status'] == expected
    assert abs(report['initial_joints'][3] - wrist) < 1e-6
    assert [entry['team'] for entry in report['results']] == ['red', 'blue']
    assert all(entry['status'] == expected for entry in report['results'])
    if expected == 'FEASIBLE':
        assert all(entry['phi_travel'] < 1e-5 for entry in report['results'])
        assert all(entry['final_joints'][3] < -6 for entry in report['results'])
    else:
        assert all(entry['diagnostics'] for entry in report['results'])


def test_ordered_targets_propagate_state_and_skip_after_failure(checker, config_file):
    code, report = run_check(
        checker, config_file, '--team', 'red', *starting_at_a(-2 * math.pi),
        '--sequence', 'shift', '--action', 'end', '--sequence', 'shift')
    assert code == 1
    assert report['mode'] == 'chain'
    first, second, third = report['results']
    assert [item['name'] for item in report['results']] == [
        'sequence:shift', 'end', 'sequence:shift']
    assert first['status'] == 'FEASIBLE'
    assert second['status'] == 'INFEASIBLE'
    assert second['initial_joints'] == first['final_joints']
    assert second['initial_joints'][3] > -0.1
    assert third['status'] == 'UNKNOWN'
    assert third['initial_joints'] is None
    assert third['final_joints'] is None
    assert third['diagnostics'][0]['code'] == 'SKIPPED_AFTER_FAILURE'


def test_all_targets_are_independent(checker, config_file):
    code, report = run_check(
        checker, config_file, '--team', 'red', '--all', *starting_at_a(-2 * math.pi))
    assert code == 2
    assert report['mode'] == 'independent'
    assert all(item['initial_joints'] == report['initial_joints'] for item in report['results'])
    results = {item['name']: item for item in report['results']}
    assert results['end']['status'] == 'FEASIBLE'
    assert results['pick:0,1']['status'] == 'FEASIBLE'
    assert results['initialize']['status'] == 'UNKNOWN'


def test_initialization_requires_explicit_final_state(checker, config_file):
    code, report = run_check(
        checker, config_file, '--team', 'red', '--action', 'initialize', 'end',
        *starting_at_a(-2 * math.pi))
    assert code == 2
    assert [entry['status'] for entry in report['results']] == ['UNKNOWN', 'UNKNOWN']
    initial = report['initial_joints']
    code, report = run_check(
        checker, config_file, '--team', 'red', '--action', 'initialize', 'end',
        *starting_at_a(-2 * math.pi), '--after-initialization-joints', *initial)
    assert code == 0
    assert [entry['status'] for entry in report['results']] == ['FEASIBLE', 'FEASIBLE']
    assert report['after_initialization_joints'] == initial
    assert report['results'][1]['initial_joints'] == initial


def test_unreachable_target_is_infeasible_with_step_diagnostic(checker, config_file):
    code, report = run_check(
        checker, config_file, '--team', 'red', '--sequence', 'unreachable', *starting_at_a(0))
    assert code == 1
    entry = report['results'][0]
    assert entry['status'] == 'INFEASIBLE'
    assert entry['diagnostics'][0]['step_index'] == 0
    assert entry['final_joints'] is None


def test_unique_initial_pose_and_explicit_joints_are_equivalent(checker, config_file):
    code, report = run_check(
        checker, config_file, '--team', 'red', '--action', 'start',
        '--initial-pose', 700, 200, 300, -1)
    assert code == 0
    initial = report['initial_joints']
    assert initial[3] < -1
    code, explicit = run_check(
        checker, config_file, '--team', 'red', '--action', 'start', '--initial-joints', *initial)
    assert code == 0
    assert explicit['initial_joints'] == initial
    assert explicit['results'][0]['final_joints'] == report['results'][0]['final_joints']


def test_warning_exit_status_can_be_enforced(checker, config_file):
    args = ['--team', 'red', '--sequence', 'shift', *starting_at_a(-2 * math.pi)]
    code, report = run_check(checker, config_file, *args)
    assert code == 0
    assert any(item['severity'].lower() == 'warning'
               for item in report['results'][0]['diagnostics'])
    code, strict = run_check(checker, config_file, *args, '--warnings-as-errors')
    assert code == 1
    assert strict['results'][0]['status'] == 'FEASIBLE'


def test_syntax_and_geometry_do_not_load_ros_middleware(checker, config_file, monkeypatch):
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_this_backend_does_not_exist')
    code, report = run_check(checker, config_file, '--syntax-only')
    assert code == 0
    assert report['status'] == 'FEASIBLE'
    code, report = run_check(
        checker, config_file, '--team', 'red', '--action', 'end', *starting_at_a(-2 * math.pi))
    assert code == 0
    assert report['results'][0]['status'] == 'FEASIBLE'


@pytest.mark.parametrize('args', [
    ['--all', '--action', 'end'],
    ['--initial-wrist', '0'],
    ['--team', 'green'],
    ['--config'],
    ['--initial-joints', '0', '1'],
    ['--initial-pose', '0', '0', '0', 'nan'],
    ['--action', 'pick:0,1junk', '--initial-pose', '700', '200', '300', '-1'],
    ['--action', 'pick:9,9', '--initial-pose', '700', '200', '300', '-1'],
])
def test_usage_errors_produce_json(checker, config_file, args):
    code, report = run_check(checker, config_file, *args)
    assert code == 2
    assert report['mode'] == 'usage_error'
    assert report['initial_message']


def test_help_describes_no_ros_and_angle_units(checker):
    result = subprocess.run([str(checker), '--help'], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0
    assert 'ROS通信には参加しません' in result.stdout
    assert 'rad' in result.stdout
    assert '--initial-wrist' in result.stdout


def test_trailing_waypoint_alone_stays_unknown(checker, continuation_file):
    code, report = run_check(
        checker, continuation_file, '--team', 'red', '--sequence', 'defer', *starting_at_a(0))
    assert code == 2
    assert report['status'] == 'UNKNOWN'
    entry = report['results'][0]
    assert entry['final_joints'] == report['initial_joints']
    assert entry['route_count'] == 0
    assert entry['pending_waypoints'] == [[720, 200, 300, 0]]
    assert entry['pending_resolution'] == ''
    assert entry['diagnostics'][0]['code'] == 'pending_waypoints'


def test_waypoint_continuation_is_checked_in_the_following_action(checker, continuation_file):
    code, report = run_check(
        checker, continuation_file, '--team', 'both', '--action', 'place:0,0',
        '--sequence', 'finish', *starting_at_a(0))
    assert code == 0
    assert report['status'] == 'FEASIBLE'
    for deferred, move in [report['results'][:2], report['results'][2:]]:
        assert deferred['status'] == move['status'] == 'FEASIBLE'
        assert deferred['route_count'] == 0
        assert '検査済み' in deferred['pending_resolution']
        assert move['initial_joints'] == deferred['final_joints']
        assert move['initial_pending_waypoints'] == deferred['pending_waypoints']
        assert move['consumed_pending_waypoints'] == 1
        assert move['pending_waypoints'] == []
        assert move['route_count'] == 1


def test_pending_waypoints_survive_mechanism_only_actions(checker, continuation_file):
    code, report = run_check(
        checker, continuation_file, '--team', 'red', '--sequence',
        'defer', 'mechanism_only', 'finish', *starting_at_a(0))
    assert code == 0
    deferred, mechanism, move = report['results']
    assert mechanism['route_count'] == 0
    assert mechanism['initial_pending_waypoints'] == deferred['pending_waypoints']
    assert mechanism['pending_waypoints'] == deferred['pending_waypoints']
    assert move['consumed_pending_waypoints'] == 1
    assert all(entry['status'] == 'FEASIBLE' for entry in report['results'])


def test_consumed_old_tail_and_new_tail_are_resolved_separately(checker, continuation_file):
    args = ['--team', 'red', '--sequence', 'defer', 'defer_again', *starting_at_a(0)]
    code, report = run_check(checker, continuation_file, *args)
    assert code == 2
    first, second = report['results']
    assert first['status'] == 'FEASIBLE'
    assert 'sequence:defer_again' in first['pending_resolution']
    assert second['status'] == 'UNKNOWN'
    assert second['consumed_pending_waypoints'] == 1
    assert second['pending_waypoints'] == [[760, 200, 300, 0]]
    code, report = run_check(checker, continuation_file, *args, '--sequence', 'finish')
    assert code == 0
    assert all(entry['status'] == 'FEASIBLE' for entry in report['results'])
    assert report['results'][2]['initial_pending_waypoints'] == [[760, 200, 300, 0]]


def test_unreachable_pending_route_fails_when_consumed(checker, continuation_file):
    code, report = run_check(
        checker, continuation_file, '--team', 'red', '--sequence',
        'far_waypoint', 'finish', *starting_at_a(0))
    assert code == 1
    assert [entry['status'] for entry in report['results']] == ['UNKNOWN', 'INFEASIBLE']
    assert report['results'][1]['initial_pending_waypoints'] == [[5000, 200, 300, 0]]
    assert report['results'][1]['diagnostics'][0]['code'] == 'unreachable_sample'


@pytest.mark.parametrize('action', ['start', 'end'])
def test_lifecycle_boundary_discards_pending_without_visiting_it(
        checker, continuation_file, action):
    code, report = run_check(
        checker, continuation_file, '--team', 'red', '--sequence', 'far_waypoint',
        '--action', action, *starting_at_a(-2 * math.pi))
    assert code == 0
    deferred, lifecycle = report['results']
    assert '破棄' in deferred['pending_resolution']
    assert lifecycle['discarded_pending_waypoints'] == 1
    assert lifecycle['consumed_pending_waypoints'] == 0
    assert lifecycle['pending_waypoints'] == []


def test_relative_tail_keeps_its_original_absolute_anchor(checker, continuation_file):
    code, report = run_check(
        checker, continuation_file, '--team', 'red', '--sequence',
        'relative_tail', 'finish', *starting_at_a(0))
    assert code == 0
    before, after = report['results']
    assert before['pending_waypoints'] == [[720, 220, 300, 0]]
    assert before['diagnostics'][-1]['step_index'] == 1
    assert after['initial_pending_waypoints'] == before['pending_waypoints']


def test_independent_place_tail_is_not_marked_feasible(checker, continuation_file):
    code, report = run_check(
        checker, continuation_file, '--team', 'red', '--all', *starting_at_a(-2 * math.pi))
    assert code == 2
    entry = next(item for item in report['results'] if item['name'] == 'place:0,0')
    assert entry['status'] == 'UNKNOWN'
    assert entry['pending_waypoints']
    assert not entry['pending_resolution']


def test_local_phi_interval_is_reported_separately_from_whole_group(checker, config_file):
    config_file.write_text(config_file.read_text().replace(
        '      - sequence_group: {start: true, max_phi_travel: 0}\n'
        '      - move: {absolute: B}\n',
        '      - sequence_group: start\n'
        '      - move: {absolute: A}\n'
        '      - phi_travel: {start: true, limit: 0.5235987755982988}\n'
        '      - move: {absolute: B}\n'
        '      - phi_travel: end\n'))
    code, report = run_check(checker, config_file, '--initial-pose', 375, 238, 196.95,
                             -math.pi, '--action', 'end')
    assert code == 0
    for entry in report['results']:
        assert entry['status'] == 'FEASIBLE'
        assert entry['phi_travel'] == pytest.approx(math.pi, abs=1e-5)
        intervals = [d for d in entry['diagnostics'] if d['code'] == 'phi_travel_interval']
        assert len(intervals) == 1
        assert '0 rad / 0.523599 rad limit' in intervals[0]['message']


def test_suction_branch_needs_live_results_even_after_syntax_passes(checker, suction_file):
    code, report = run_check(checker, suction_file, '--syntax-only')
    assert code == 0
    assert report['status'] == 'FEASIBLE'
    code, report = run_check(
        checker, suction_file, '--team', 'red', '--sequence', 'inspect_suction',
        *starting_at_a(0))
    assert code == 2
    assert report['status'] == 'UNKNOWN'
    entry = report['results'][0]
    assert entry['status'] == 'UNKNOWN'
    assert entry['final_joints'] is None
    assert entry['route_count'] == 0
    assert entry['pending_waypoints'] == []
    assert 'live pressure' in entry['message']
    assert entry['diagnostics'][-1]['code'] == 'suction_result_unknown'
    assert entry['diagnostics'][-1]['severity'] == 'unknown'


def test_suction_unknown_cannot_resolve_prior_pending_or_resume_chain(checker, suction_file):
    code, report = run_check(
        checker, suction_file, '--team', 'red', '--sequence',
        'defer', 'inspect_suction', 'finish', *starting_at_a(0))
    assert code == 2
    deferred, inspected, finish = report['results']
    assert [entry['status'] for entry in report['results']] == ['UNKNOWN'] * 3
    assert deferred['pending_resolution'] == ''
    assert inspected['initial_pending_waypoints'] == deferred['pending_waypoints']
    assert inspected['consumed_pending_waypoints'] == 0
    assert inspected['pending_waypoints'] == []
    assert inspected['final_joints'] is None
    assert inspected['diagnostics'][-1]['code'] == 'suction_result_unknown'
    assert finish['initial_joints'] is None
    assert finish['route_count'] == 0
    assert finish['diagnostics'][0]['code'] == 'SKIPPED_AFTER_FAILURE'


def test_checked_prefix_resolves_old_waypoints_but_not_suction_completion(checker, suction_file):
    code, report = run_check(
        checker, suction_file, '--team', 'red', '--sequence',
        'defer', 'inspect_after_move', 'finish', *starting_at_a(0))
    assert code == 2
    deferred, inspected, finish = report['results']
    assert deferred['status'] == 'FEASIBLE'
    assert 'sequence:inspect_after_move' in deferred['pending_resolution']
    assert inspected['status'] == 'UNKNOWN'
    assert inspected['route_count'] == 1
    assert inspected['consumed_pending_waypoints'] == 1
    assert inspected['pending_waypoints'] == []
    assert inspected['pending_resolution'] == ''
    assert inspected['final_joints'] is None
    assert finish['status'] == 'UNKNOWN'
    assert finish['initial_joints'] is None
    assert finish['diagnostics'][0]['code'] == 'SKIPPED_AFTER_FAILURE'


def test_unconditional_break_skips_failure_and_preserves_following_move(checker, suction_file):
    code, report = run_check(
        checker, suction_file, '--team', 'red', '--sequence', 'break_before_failure',
        *starting_at_a(0))
    assert code == 0
    entry = report['results'][0]
    assert entry['status'] == 'FEASIBLE'
    assert entry['route_count'] == 1
    assert entry['final_joints'] is not None
    assert all(diagnostic['code'] != 'explicit_fail' for diagnostic in entry['diagnostics'])


def test_reached_failure_blocks_following_sequence(checker, suction_file):
    code, report = run_check(
        checker, suction_file, '--team', 'red', '--sequence',
        'intentional_failure', 'finish', *starting_at_a(0))
    assert code == 1
    failed, following = report['results']
    assert failed['status'] == 'INFEASIBLE'
    assert failed['route_count'] == 0
    assert failed['final_joints'] is None
    assert failed['diagnostics'][0]['code'] == 'explicit_fail'
    assert failed['diagnostics'][0]['message'] == 'Explicit failure'
    assert following['status'] == 'UNKNOWN'
    assert following['diagnostics'][0]['code'] == 'SKIPPED_AFTER_FAILURE'

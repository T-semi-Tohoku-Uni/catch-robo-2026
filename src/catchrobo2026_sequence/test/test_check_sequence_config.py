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

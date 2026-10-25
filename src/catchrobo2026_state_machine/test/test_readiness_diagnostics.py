"""Keep unavailable transport status separate from valid arm diagnostics."""
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from canrpc_bridge.srv import Call
from catchrobo2026_msgs.srv import ArmTargetGate
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger


@pytest.fixture
def diagnostics(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '210')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('diagnostic_fixture', namespace=f'/diag_{os.getpid()}')
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    class Rig:
        events = []
        outcome, result, flags, version, length = 4, 2, 0, 3, 64
        initialize_outcome, initialize_result, initialize_version, initialize_length = 0, 1, 3, 64
        boot = bytes([1]) + bytes(15)
        initialize_boot = bytes([1]) + bytes(15)
        ready_sequence = []
        reason = 'startup incomplete: arm node did not become READY'

        def query(self):
            self.events.clear()
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + 4
            while not future.done():
                assert child.poll() is None
                assert time.monotonic() < deadline
                executor.spin_once(timeout_sec=0.01)
            return future.result()

    rig = Rig()

    def gate(request, response):
        rig.events.append('pause' if request.pause else 'resume')
        assert request.deadline_steady_ns > time.monotonic_ns()
        if not request.pause:
            assert request.token == 42
        response.success = True
        response.token = 42
        response.message = 'gate mock'
        return response

    def rpc(request, response):
        initialize = request.endpoint == 'arm_initialize'
        rig.events.append('initialize' if initialize else 'rpc')
        assert request.endpoint in ('arm_initialize', 'arm_ready') and not request.data
        response.outcome = rig.initialize_outcome if initialize else rig.outcome
        response.message = rig.reason
        result, flags, stage = ((rig.initialize_result, 0, 0) if initialize
                                else (rig.result, rig.flags, 255))
        if not initialize and rig.ready_sequence:
            result, flags, stage = rig.ready_sequence.pop(0)
        version = rig.initialize_version if initialize else rig.version
        length = rig.initialize_length if initialize else rig.length
        boot = rig.initialize_boot if initialize else rig.boot
        header = struct.pack('<IIBBBB', result, flags, stage, 9, 255, version)
        axis = struct.pack('<BBBBIBBH', 255, 255, 143, 255, 0xffffffff, 15, 3, 0)
        response.data = list((header + axis * 3 + boot)[:length])
        return response

    node.create_service(ArmTargetGate, 'arm_target/gate', gate)
    node.create_service(Call, 'canrpc/call', rpc)
    binary = (Path(get_package_prefix('catchrobo2026_state_machine')) /
              'lib/catchrobo2026_state_machine/state_machine_node')
    log_path = tmp_path / 'state.log'
    with log_path.open('w') as log:
        child = subprocess.Popen([str(binary), '--ros-args', '-r',
                                  '__ns:=' + node.get_namespace(),
                                  '-p', 'rpc_timeout_sec:=0.45',
                                  '-p', 'readiness_poll_period_sec:=0.03'],
                                 stdout=log, stderr=log, start_new_session=True)
        try:
            client = node.create_client(Trigger, 'request_initialization')
            assert client.wait_for_service(timeout_sec=8)
            deadline = time.monotonic() + 0.35
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
            yield rig
        finally:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
    print(log_path.read_text())


def assert_all_axes(message):
    assert all(f'q{axis} ' in message for axis in range(3)), message
    assert message.count('cmd=CURRENT_REFERENCE') == 3
    assert message.count('rsp=RESPONSE_WAIT') == 3


@pytest.mark.parametrize('outcome', [1, 2, 3, 4, 5, 6])
def test_non_ok_ready_payload_never_resumes(diagnostics, outcome):
    diagnostics.outcome, diagnostics.result, diagnostics.flags = outcome, 0, 127
    response = diagnostics.query()
    assert not response.success
    assert diagnostics.events == ['pause', 'initialize', 'rpc']
    assert_all_axes(response.message)
    assert f'CANRPC[{outcome}]' in response.message
    assert diagnostics.reason in response.message


def test_unavailable_fault_diagnostics_survive_sequence_ui_budget(diagnostics):
    diagnostics.reason = 'startup incomplete: ' + 'failure reason ' * 40
    response = diagnostics.query()
    assert not response.success
    assert diagnostics.events == ['pause', 'initialize', 'rpc']
    assert_all_axes(response.message)
    assert 'startup incomplete:' in response.message
    assert 'INITIALIZATION_TIMEOUT' in response.message
    assert 'snapshot(age@failure)=1' in response.message
    sequence_message = ('initialization sequence failed: initialization rejected: '
                        + response.message)
    assert len(sequence_message.encode()) < 512


@pytest.mark.parametrize('version,length', [(1, 64), (2, 8), (1, 48)])
def test_invalid_payload_is_not_decoded(diagnostics, version, length):
    diagnostics.version, diagnostics.length = version, length
    response = diagnostics.query()
    assert not response.success
    assert diagnostics.events == ['pause', 'initialize', 'rpc']
    assert 'raw=' in response.message
    assert 'q0 ' not in response.message
    assert diagnostics.reason in response.message


def test_only_normal_ok_ready_may_resume_after_failure(diagnostics):
    diagnostics.result, diagnostics.flags = 0, 127
    assert not diagnostics.query().success
    assert diagnostics.events == ['pause', 'initialize', 'rpc']
    diagnostics.outcome = 0
    assert diagnostics.query().success
    assert diagnostics.events == ['pause', 'initialize', 'rpc', 'resume']
    diagnostics.result, diagnostics.flags = 2, 0
    assert not diagnostics.query().success
    assert diagnostics.events == ['pause', 'initialize', 'rpc']


def test_running_polls_same_boot_once_started(diagnostics):
    diagnostics.outcome = 0
    diagnostics.ready_sequence = [(1, 0, 0), (1, 0, 15), (0, 127, 29)]
    assert diagnostics.query().success
    assert diagnostics.events == ['pause', 'initialize', 'rpc', 'rpc', 'rpc', 'resume']


def test_boot_change_during_poll_never_resumes(diagnostics):
    diagnostics.outcome, diagnostics.result, diagnostics.flags = 0, 0, 127
    diagnostics.boot = bytes([2]) + bytes(15)
    response = diagnostics.query()
    assert not response.success and 'boot changed' in response.message
    assert diagnostics.events == ['pause', 'initialize', 'rpc']


@pytest.mark.parametrize('failure', ['unavailable', 'lost', 'old48', 'zero_boot'])
def test_start_failure_only_reads_diagnostic_once_and_never_restarts(diagnostics, failure):
    diagnostics.outcome, diagnostics.result, diagnostics.flags = 0, 0, 127
    if failure == 'unavailable':
        diagnostics.initialize_outcome, diagnostics.initialize_length = 4, 0
    elif failure == 'lost':
        diagnostics.initialize_outcome, diagnostics.initialize_length = 2, 0
    elif failure == 'old48':
        diagnostics.initialize_version, diagnostics.initialize_length = 1, 48
    else:
        diagnostics.initialize_boot = bytes(16)
    response = diagnostics.query()
    assert not response.success
    assert_all_axes(response.message)
    assert diagnostics.events == ['pause', 'initialize', 'rpc']
    assert 'init: ' in response.message


def test_failed_start_and_missing_fallback_keep_original_failure(diagnostics):
    diagnostics.initialize_outcome, diagnostics.initialize_length = 4, 0
    diagnostics.outcome, diagnostics.length = 4, 0
    response = diagnostics.query()
    assert not response.success and diagnostics.reason in response.message
    assert diagnostics.events == ['pause', 'initialize', 'rpc']


def test_zero_boot_and_old_format_ready_are_never_accepted(diagnostics):
    diagnostics.outcome, diagnostics.result, diagnostics.flags = 0, 0, 127
    diagnostics.boot = bytes(16)
    assert not diagnostics.query().success
    assert diagnostics.events == ['pause', 'initialize', 'rpc']


def test_running_deadline_never_resumes_or_restarts(diagnostics):
    diagnostics.outcome, diagnostics.result, diagnostics.flags = 0, 1, 0
    response = diagnostics.query()
    assert not response.success and 'timeout' in response.message
    assert_all_axes(response.message)
    assert diagnostics.events.count('initialize') == 1
    assert 'resume' not in diagnostics.events


@pytest.mark.parametrize('version,flags', [(2, 127), (3, 31), (3, 63)])
def test_ack_or_position_without_settling_never_resumes(diagnostics, version, flags):
    diagnostics.outcome, diagnostics.result = 0, 0
    diagnostics.version, diagnostics.flags = version, flags
    response = diagnostics.query()
    assert not response.success
    assert diagnostics.events.count('initialize') == 1
    assert 'resume' not in diagnostics.events


def test_resume_waits_until_motion_and_dwell_flags(diagnostics):
    diagnostics.outcome = 0
    diagnostics.ready_sequence = [(0, 31, 32), (0, 63, 33), (0, 127, 29)]
    assert diagnostics.query().success
    assert diagnostics.events == ['pause', 'initialize', 'rpc', 'rpc', 'rpc', 'resume']

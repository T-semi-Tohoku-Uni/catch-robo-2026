"""Check bounded robot-specific target RPC and local gate epochs without CAN."""
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from canrpc_bridge.msg import BridgeStatus
from canrpc_bridge.srv import Call
from catchrobo2026_msgs.srv import ArmTargetGate
import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger


@pytest.fixture
def rig(tmp_path, monkeypatch, request):
    monkeypatch.setenv('ROS_DOMAIN_ID', '210')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node(
        'target_test', namespace=f'/target_{os.getpid()}_{time.monotonic_ns()}')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    log = (tmp_path / 'target.log').open('w')
    binary = (Path(get_package_prefix('catchrobo2026_state_machine')) /
              'lib/catchrobo2026_state_machine/arm_target_client')
    options = getattr(request, 'param', {})
    options = options if isinstance(options, dict) else {'period_ms': options}
    extra = []
    if options.get('trajectory'):
        dynamics = tmp_path / 'synthetic_dynamics.yaml'
        dynamics.write_text("""robot_dynamics:
  source: synthetic_host_test_only
  measured_at: synthetic_not_robot_measurement
  safety_factor: 0.85
  joint_peak_torque: [100.0, 100.0, 100.0, 100.0]
  links:
""" + ''.join("""    - mass: 0.6
      center_of_mass: [0.0, 0.0, 0.15]
      inertia: [0.015, 0.0, 0.0, 0.015, 0.0, 0.25]
""" for _ in range(5)) + """  theta1:
    continuous_torque: 50.0
    peak_torque: 80.0
    gear_ratio: 10.0
    efficiency: 0.9
    positive_speed_torque_curve: [[0.0, 80.0], [10.0, 80.0]]
    negative_speed_torque_curve: [[0.0, 80.0], [10.0, 80.0]]
""")
        pipeline = tmp_path / 'synthetic_pipeline.yaml'
        pipeline.write_text("""trajectory_pipeline:
  hardware_output_enabled: true
  active_axes: 3
  distance_weights: [2.0, 1.0, 1.0, 0.0]
  max_velocity: [2.0, 2.0, 2.0, 2.0]
  max_acceleration: [5.0, 5.0, 5.0, 5.0]
  max_jerk: [100.0, 100.0, 100.0, 100.0]
  planning_time_limit: 0.3
  candidate_count: 1
""")
        extra = ['-p', f'trajectory_dynamics_config:={dynamics}',
                 '-p', f'trajectory_pipeline_config:={pipeline}',
                 '-p', 'trajectory_timeout_sec:=3.0']
    status_timeout = options.get('status_timeout', 2.5)
    child = subprocess.Popen([str(binary), '--ros-args', '-r', f'__ns:={node.get_namespace()}',
                              '-p', 'canrpc_service:=fake/call', '-p', 'rpc_timeout_sec:=0.15',
                              '-p', f'target_period_ms:={options.get("period_ms", 10)}',
                              '-p', f'bridge_status_timeout_sec:={status_timeout}'] + extra,
                             stdout=log, stderr=log, start_new_session=True)

    class Rig:
        records, waiters = [], []
        hold_final = False
        hold, outcome, result, mask, mismatch = False, 0, 0, 7, False
        actual = [0.2, 0.3, 0.4]
        follow = False
        feedback_enabled = True
        rebase_ok = True
        rebase_hold = False
        rebase_waiters = []
        status_enabled = True
        status_ready = True
        status_session = '0123456789abcdef' * 2

        def spin(self, seconds=0.05):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                assert child.poll() is None
                executor.spin_once(timeout_sec=0.002)

        def until(self, predicate, timeout=4):
            deadline = time.monotonic() + timeout
            while not predicate():
                assert time.monotonic() < deadline
                self.spin(0.005)

        def resolve(self, future):
            self.until(future.done)
            return future.result()

        def gate(self, pause=True, token=0, seconds=1):
            return self.client.call_async(ArmTargetGate.Request(
                pause=pause, token=token,
                deadline_steady_ns=time.monotonic_ns() + int(seconds * 1e9)))

        def resume(self):
            paused = self.resolve(self.gate())
            assert paused.success
            assert self.resolve(self.gate(False, paused.token)).success
            self.until(lambda: self.publisher.get_subscription_count() == 1)
            self.spin(0.05)

        def send(self, values):
            self.publisher.publish(Float32MultiArray(data=values))

        def release(self):
            pending, self.waiters = list(self.waiters), []
            for future in pending:
                if not future.done():
                    future.set_result(True)
    harness = Rig()
    harness.status_enabled = options.get('status_enabled', True)
    group = ReentrantCallbackGroup()

    async def call(request, response):
        harness.records.append((time.monotonic(), bytes(request.data), request.endpoint))
        held, outcome, result, mask, mismatch = (harness.hold, harness.outcome, harness.result,
                                                 harness.mask, harness.mismatch)
        if harness.hold_final and abs(struct.unpack('<3f', bytes(request.data))[0] - 0.21) < 1e-7:
            held = True
        if held:
            future = Future(executor=executor)
            harness.waiters.append(future)
            await future
        if harness.follow and outcome == 0 and result == 0:
            harness.actual = list(struct.unpack('<3f', bytes(request.data)))
        response.outcome = outcome
        echo = bytearray(request.data)
        if mismatch:
            echo[0] ^= 1
        response.data = list(struct.pack('<I', result) + echo + struct.pack('<I', mask))
        response.message = 'native command mock'
        return response
    node.create_service(Call, 'fake/call', call, callback_group=group)

    async def rebase(_request, response):
        if harness.rebase_hold:
            future = Future(executor=executor)
            harness.rebase_waiters.append(future)
            await future
        response.success = harness.rebase_ok
        return response

    node.create_service(Trigger, 'joy/rebase', rebase, callback_group=group)

    harness.child = child
    harness.node = node
    harness.current = node.create_publisher(Float32MultiArray, 'current_joints', 1)
    status = node.create_publisher(BridgeStatus, 'canrpc/status', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def feedback():
        if harness.status_enabled:
            status.publish(BridgeStatus(
                state=BridgeStatus.READY if harness.status_ready else BridgeStatus.DEGRADED,
                commands_enabled=harness.status_ready, socket_open=harness.status_ready,
                session_id=harness.status_session, reason='host transport mock'))
        if harness.feedback_enabled:
            harness.current.publish(Float32MultiArray(data=harness.actual))

    node.create_timer(0.01, feedback)
    harness.client = node.create_client(ArmTargetGate, 'arm_target/gate')
    harness.publisher = node.create_publisher(Float32MultiArray, 'target_joint_angles', 10)
    assert harness.client.wait_for_service(timeout_sec=10)
    harness.until(lambda: harness.publisher.get_subscription_count() == 1)
    harness.spin(0.35)
    try:
        yield harness
    finally:
        harness.release()
        for pending in harness.rebase_waiters:
            if not pending.done():
                pending.set_result(True)
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
        log.close()
        print((tmp_path / 'target.log').read_text())


def test_start_paused_and_resume_discards_preexisting_input(rig):
    rig.send([0.21, 0.3, 0.4])
    rig.spin(0.1)
    assert not rig.records
    rig.resume()
    rig.spin(0.1)
    assert not rig.records
    rig.send([0.22, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 1)
    assert rig.records[0][1:] == (struct.pack('<3f', 0.22, 0.3, 0.4), 'arm_target')


def test_gate_rejects_expired_resume_and_stale_pause_token(rig):
    first = rig.resolve(rig.gate())
    second = rig.resolve(rig.gate())
    assert first.success and second.success and first.token != second.token
    assert not rig.resolve(rig.gate(False, first.token)).success
    assert not rig.resolve(rig.gate(False, second.token, seconds=-1)).success
    rig.send([0.21, 0.3, 0.4])
    rig.spin()
    assert not rig.records
    assert rig.resolve(rig.gate(False, second.token)).success


def test_expired_pause_still_closes_gate(rig):
    rig.resume()
    reply = rig.resolve(rig.gate(seconds=-1))
    assert not reply.success
    rig.send([0.21, 0.3, 0.4])
    rig.spin()
    assert not rig.records


def test_latest_one_coalesces_while_single_rpc_is_inflight(rig):
    rig.resume()
    rig.hold = True
    rig.send([0.2, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 1)
    for index in range(30):
        rig.send([0.2 + float(index) / 1000., 0.3, 0.4])
        rig.spin(0.002)
    assert len(rig.records) == 1
    rig.hold = False
    rig.release()
    rig.until(lambda: len(rig.records) == 2)
    assert rig.records[1][1] == struct.pack('<3f', 0.229, 0.3, 0.4)
    rig.spin(0.05)
    assert len(rig.records) == 2


def test_pause_waits_for_inflight_and_resume_requires_new_input(rig):
    rig.resume()
    rig.hold = True
    rig.send([0.21, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 1)
    paused = rig.gate()
    rig.spin(0.03)
    assert not paused.done()
    rig.send([0.22, 0.3, 0.4])
    rig.hold = False
    rig.release()
    reply = rig.resolve(paused)
    assert reply.success
    assert rig.resolve(rig.gate(False, reply.token)).success
    rig.spin(0.1)
    assert len(rig.records) == 1
    rig.send([0.23, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 2)
    assert rig.records[1][1] == struct.pack('<3f', 0.23, 0.3, 0.4)


@pytest.mark.parametrize('outcome,result,mask,mismatch', [
    (1, 0, 7, False), (2, 0, 7, False), (3, 0, 7, False),
    (4, 0, 7, False), (5, 0, 7, False), (0, 1, 7, False),
    (0, 0, 3, False), (0, 0, 7, True),
])
def test_unconfirmed_rpc_latches_until_explicit_pause_resume(rig, outcome, result, mask, mismatch):
    rig.resume()
    rig.outcome, rig.result, rig.mask, rig.mismatch = outcome, result, mask, mismatch
    rig.send([0.21, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 1)
    rig.spin(0.03)
    for _ in range(10):
        rig.send([0.21, 0.3, 0.4])
        rig.spin(0.002)
    assert len(rig.records) == 1
    rig.outcome, rig.result, rig.mask, rig.mismatch = 0, 0, 7, False
    rig.resume()
    rig.spin(0.05)
    assert len(rig.records) == 1
    rig.send([0.22, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 2)


def test_timeout_latches_and_late_completion_cannot_restore_stream(rig):
    rig.resume()
    rig.hold = True
    rig.send([0.21, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 1)
    rig.spin(0.2)
    rig.release()
    for _ in range(15):
        rig.send([0.22, 0.3, 0.4])
        rig.spin(0.002)
    assert len(rig.records) == 1


def test_500hz_input_never_exceeds_100hz_send_and_invalid_arrays_are_ignored(rig):
    rig.resume()
    for values in ([1., 2.], [1., 2., 3., 4.], [float('nan'), 1., 2.]):
        rig.send(values)
        rig.spin(0.02)
    assert not rig.records
    began = time.monotonic()
    while time.monotonic() - began < 0.25:
        rig.send([0.2 + float(len(rig.records)) / 1000., 0.3, 0.4])
        rig.spin(0.002)
    rig.spin(0.03)
    assert 2 <= len(rig.records) <= 27
    assert all(second[0] - first[0] >= 0.008
               for first, second in zip(rig.records, rig.records[1:]))


@pytest.mark.parametrize('rig', [25], indirect=True)
def test_configured_period_reduces_send_rate_without_changing_input_rate(rig):
    rig.resume()
    began = time.monotonic()
    while time.monotonic() - began < 0.25:
        rig.send([0.2 + float(len(rig.records)) / 1000., 0.3, 0.4])
        rig.spin(0.002)
    rig.spin(0.03)
    assert 2 <= len(rig.records) <= 13
    assert all(second[0] - first[0] >= 0.020
               for first, second in zip(rig.records, rig.records[1:]))


def test_idle_bridge_fault_closes_open_gate_until_explicit_init(rig):
    rig.resume()
    rig.status_ready = False
    rig.spin(0.05)
    rig.status_ready = True
    rig.spin(0.05)
    rig.send([0.21, 0.3, 0.4])
    rig.spin(0.05)
    assert not rig.records
    rig.resume()
    rig.spin(0.05)
    assert not rig.records
    rig.send([0.2, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 1)


def test_ready_session_change_closes_idle_gate_without_replaying(rig):
    rig.resume()
    rig.status_session = 'fedcba9876543210' * 2
    rig.spin(0.05)
    rig.send([0.21, 0.3, 0.4])
    rig.spin(0.05)
    assert not rig.records
    rig.resume()
    rig.spin(0.05)
    assert not rig.records
    rig.send([0.2, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 1)


@pytest.mark.parametrize('rig', [{'status_timeout': 1.1}], indirect=True)
def test_expired_status_rejects_resume_and_ready_does_not_reopen(rig):
    rig.resume()
    rig.status_enabled = False
    rig.spin(1.25)
    paused = rig.resolve(rig.gate())
    rejected = rig.resolve(rig.gate(False, paused.token))
    assert not rejected.success and 'heartbeat expired' in rejected.message
    rig.status_enabled = True
    rig.spin(0.05)
    rig.send([0.21, 0.3, 0.4])
    rig.spin(0.05)
    assert not rig.records
    rig.resume()
    rig.send([0.2, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 1)


def test_status_failure_invalidates_pending_resume_and_late_rebase(rig):
    paused = rig.resolve(rig.gate())
    rig.rebase_hold = True
    resumed = rig.gate(False, paused.token)
    rig.until(lambda: bool(rig.rebase_waiters))
    rig.status_ready = False
    assert not rig.resolve(resumed).success
    rig.rebase_waiters.pop().set_result(True)
    rig.rebase_hold = False
    rig.status_ready = True
    rig.spin(0.05)
    assert not rig.resolve(rig.gate(False, paused.token)).success
    rig.send([0.21, 0.3, 0.4])
    rig.spin(0.05)
    assert not rig.records
    rig.resume()
    rig.send([0.2, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 1)


@pytest.mark.parametrize('rig', [{'status_enabled': False}], indirect=True)
def test_missing_status_cannot_open_gate_and_first_ready_needs_init(rig):
    paused = rig.resolve(rig.gate())
    rejected = rig.resolve(rig.gate(False, paused.token))
    assert not rejected.success and 'status not received' in rejected.message
    rig.send([0.21, 0.3, 0.4])
    rig.spin(0.05)
    assert not rig.records
    rig.status_enabled = True
    rig.spin(0.05)
    rig.send([0.21, 0.3, 0.4])
    rig.spin(0.05)
    assert not rig.records
    rig.resume()
    rig.send([0.2, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 1)


@pytest.mark.parametrize('rig', [{'status_timeout': 1.1}], indirect=True)
def test_same_ready_after_executor_gap_cannot_hide_expired_status(rig):
    rig.resume()
    os.kill(rig.child.pid, signal.SIGSTOP)
    try:
        rig.spin(1.25)
    finally:
        os.kill(rig.child.pid, signal.SIGCONT)
    rig.spin(0.1)
    rig.send([0.21, 0.3, 0.4])
    rig.spin(0.05)
    assert not rig.records
    rig.resume()
    rig.send([0.2, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 1)


@pytest.mark.parametrize('target', [[0.2 + 6.2831853, 0.3, 0.4], [0.0307, 1.865, 4.309]])
def test_first_handoff_rejects_nonwrapped_jump(rig, target):
    rig.resume()
    rig.send(target)
    rig.spin(0.12)
    assert not rig.records
    rig.send([0.2, 0.3, 0.4])
    rig.spin(0.05)
    assert not rig.records  # Rejection keeps the existing explicit-init latch.


def test_first_handoff_preserves_measured_turn(rig):
    rig.actual = [0.2 + 6.2831853, 0.3, 0.4]
    rig.spin(0.08)
    rig.resume()
    rig.send(list(rig.actual))
    rig.until(lambda: len(rig.records) == 1)
    assert rig.records[0][1] == struct.pack('<3f', *rig.actual)


def test_handoff_requires_fresh_feedback_without_ros_arrival_judgement(rig):
    rig.feedback_enabled = False
    rig.spin(0.6)
    paused = rig.resolve(rig.gate())
    assert paused.success
    assert not rig.resolve(rig.gate(False, paused.token)).success
    assert not rig.records


def test_logged_feedback_can_resume_and_send_unchanged_first_target(rig):
    rig.actual = [0.0307, -1.9746, 1.8650]
    rig.spin(0.08)
    rig.resume()
    rig.send(list(rig.actual))
    rig.until(lambda: len(rig.records) == 1)
    assert rig.records[0][1] == struct.pack('<3f', *rig.actual)


def test_logged_ik_candidate_rejected_as_first_handoff_target(rig):
    rig.actual = [0.0307, -1.9746, 1.8650]
    rig.spin(0.08)
    rig.resume()
    rig.send([0.0307, 1.8650, 4.309])
    rig.spin(0.12)
    assert not rig.records


def test_normal_target_keeps_existing_finite_rpc_contract(rig):
    rig.resume()
    rig.send([0.2, 0.3, 0.4])
    rig.until(lambda: len(rig.records) == 1)
    rig.spin(0.03)
    target = [0.0307, -1.9746, 1.8650]
    rig.send(target)
    rig.until(lambda: len(rig.records) == 2)
    assert rig.records[1][1] == struct.pack('<3f', *target)


def test_handoff_does_not_rejudge_mcu_initial_pose(rig):
    # The state service owns the MCU final response; the gate preserves live q.
    rig.actual = [0.0, 1.0, 0.0]
    rig.spin(0.08)
    rig.resume()
    rig.send(list(rig.actual))
    rig.until(lambda: len(rig.records) == 1)
    assert rig.records[0][1] == struct.pack('<3f', *rig.actual)

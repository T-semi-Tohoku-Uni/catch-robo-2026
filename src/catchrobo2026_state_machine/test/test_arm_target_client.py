"""Check bounded robot-specific target RPC and local gate epochs without CAN."""
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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger


@pytest.fixture
def rig(tmp_path, monkeypatch, request):
    monkeypatch.setenv('ROS_DOMAIN_ID', '207')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
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
    child = subprocess.Popen([str(binary), '--ros-args', '-r', f'__ns:={node.get_namespace()}',
                              '-p', 'canrpc_service:=fake/call', '-p', 'rpc_timeout_sec:=0.15',
                              '-p', f'target_period_ms:={options.get("period_ms", 10)}'] + extra,
                             stdout=log, stderr=log, start_new_session=True)

    class Rig:
        records, waiters = [], []
        hold_final = False
        hold, outcome, result, mask, mismatch = False, 0, 0, 7, False
        actual = [0.2, 0.3, 0.4]
        follow = False
        feedback_enabled = True
        rebase_ok = True

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

    def rebase(_request, response):
        response.success = harness.rebase_ok
        return response

    node.create_service(Trigger, 'joy/rebase', rebase)

    harness.node = node
    harness.current = node.create_publisher(Float32MultiArray, 'current_joints', 1)

    def feedback():
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
    rig.send([1., 2., 3.])
    rig.spin(0.1)
    assert not rig.records
    rig.resume()
    rig.spin(0.1)
    assert not rig.records
    rig.send([4., 5., 6.])
    rig.until(lambda: len(rig.records) == 1)
    assert rig.records[0][1:] == (struct.pack('<3f', 4, 5, 6), 'arm_target')


def test_gate_rejects_expired_resume_and_stale_pause_token(rig):
    first = rig.resolve(rig.gate())
    second = rig.resolve(rig.gate())
    assert first.success and second.success and first.token != second.token
    assert not rig.resolve(rig.gate(False, first.token)).success
    assert not rig.resolve(rig.gate(False, second.token, seconds=-1)).success
    rig.send([1., 2., 3.])
    rig.spin()
    assert not rig.records
    assert rig.resolve(rig.gate(False, second.token)).success


def test_expired_pause_still_closes_gate(rig):
    rig.resume()
    reply = rig.resolve(rig.gate(seconds=-1))
    assert not reply.success
    rig.send([1., 2., 3.])
    rig.spin()
    assert not rig.records


def test_latest_one_coalesces_while_single_rpc_is_inflight(rig):
    rig.resume()
    rig.hold = True
    rig.send([0., 0., 0.])
    rig.until(lambda: len(rig.records) == 1)
    for index in range(30):
        rig.send([float(index), 1., 2.])
        rig.spin(0.002)
    assert len(rig.records) == 1
    rig.hold = False
    rig.release()
    rig.until(lambda: len(rig.records) == 2)
    assert rig.records[1][1] == struct.pack('<3f', 29, 1, 2)
    rig.spin(0.05)
    assert len(rig.records) == 2


def test_pause_waits_for_inflight_and_resume_requires_new_input(rig):
    rig.resume()
    rig.hold = True
    rig.send([1., 2., 3.])
    rig.until(lambda: len(rig.records) == 1)
    paused = rig.gate()
    rig.spin(0.03)
    assert not paused.done()
    rig.send([4., 5., 6.])
    rig.hold = False
    rig.release()
    reply = rig.resolve(paused)
    assert reply.success
    assert rig.resolve(rig.gate(False, reply.token)).success
    rig.spin(0.1)
    assert len(rig.records) == 1
    rig.send([7., 8., 9.])
    rig.until(lambda: len(rig.records) == 2)
    assert rig.records[1][1] == struct.pack('<3f', 7, 8, 9)


@pytest.mark.parametrize('outcome,result,mask,mismatch', [
    (1, 0, 7, False), (2, 0, 7, False), (3, 0, 7, False),
    (4, 0, 7, False), (5, 0, 7, False), (0, 1, 7, False),
    (0, 0, 3, False), (0, 0, 7, True),
])
def test_unconfirmed_rpc_latches_until_explicit_pause_resume(rig, outcome, result, mask, mismatch):
    rig.resume()
    rig.outcome, rig.result, rig.mask, rig.mismatch = outcome, result, mask, mismatch
    rig.send([1., 2., 3.])
    rig.until(lambda: len(rig.records) == 1)
    rig.spin(0.03)
    for _ in range(10):
        rig.send([1., 2., 3.])
        rig.spin(0.002)
    assert len(rig.records) == 1
    rig.outcome, rig.result, rig.mask, rig.mismatch = 0, 0, 7, False
    rig.resume()
    rig.spin(0.05)
    assert len(rig.records) == 1
    rig.send([4., 5., 6.])
    rig.until(lambda: len(rig.records) == 2)


def test_timeout_latches_and_late_completion_cannot_restore_stream(rig):
    rig.resume()
    rig.hold = True
    rig.send([1., 2., 3.])
    rig.until(lambda: len(rig.records) == 1)
    rig.spin(0.2)
    rig.release()
    for _ in range(15):
        rig.send([4., 5., 6.])
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
        rig.send([float(len(rig.records)), 1., 2.])
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
        rig.send([float(len(rig.records)), 1., 2.])
        rig.spin(0.002)
    rig.spin(0.03)
    assert 2 <= len(rig.records) <= 13
    assert all(second[0] - first[0] >= 0.020
               for first, second in zip(rig.records, rig.records[1:]))

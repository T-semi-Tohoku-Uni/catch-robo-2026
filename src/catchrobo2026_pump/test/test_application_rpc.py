"""Check remote application confirmation through real ROS app services."""
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from canrpc_bridge.srv import Call
from catchrobo2026_msgs.srv import ArmTargetGate, PumpControl
import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger


@pytest.fixture
def rig(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '223')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    rclpy.init()
    node = rclpy.create_node('application_rpc_test', namespace=f'/app_rpc_{os.getpid()}_{time.monotonic_ns()}')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    children, timers, pending = [], [], []
    log = (tmp_path / 'apps.log').open('w')
    class Rig:
        requests, status, legacy = [], [], []
        delay, outcome, app_result, flags, mismatch, raw = 0.0, 0, 0, 31, False, None
        def spin(self, duration=0.1):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                assert all(child.poll() is None for child in children)
                executor.spin_once(timeout_sec=0.005)
        def until(self, condition, timeout=5):
            deadline = time.monotonic() + timeout
            while not condition():
                assert time.monotonic() < deadline
                self.spin(0.01)
        def result(self, future):
            self.until(future.done)
            return future.result()
        def launch(self, timeout=0.2, selected=1):
            for package, binary in [('catchrobo2026_pump', 'pump_controller_node'),
                                    ('catchrobo2026_state_machine', 'state_machine_node')]:
                path = Path(get_package_prefix(package)) / 'lib' / package / binary
                child = subprocess.Popen([str(path), '--ros-args', '-r', f'__ns:={node.get_namespace()}',
                    '-p', 'canrpc_service:=test_rpc/call', '-p', f'rpc_timeout_sec:={timeout}',
                    '-p', f'selected_pump:={selected}'], stdout=log, stderr=log, start_new_session=True)
                children.append(child)
            assert self.pump.wait_for_service(timeout_sec=10)
            assert self.ready.wait_for_service(timeout_sec=10)
            self.spin(0.25)
        def set(self, values=(1, 0, 0)):
            return self.pump.call_async(PumpControl.Request(left=values[0], center=values[1], right=values[2]))
    harness = Rig()
    group = ReentrantCallbackGroup()
    async def rpc(request, response):
        harness.requests.append(request)
        delay, outcome, result, flags, mismatch, raw = harness.delay, harness.outcome, harness.app_result, harness.flags, harness.mismatch, harness.raw
        if delay:
            future = Future(executor=executor)
            pending.append(future)
            def finish():
                timer.cancel()
                if not future.done(): future.set_result(True)
            timer = node.create_timer(delay, finish, callback_group=group)
            timers.append(timer)
            await future
        response.outcome = outcome
        mask = struct.unpack('<I', bytes(request.data))[0] if request.data else flags
        data = struct.pack('<II', result, mask ^ int(mismatch))
        if request.endpoint in ('arm_initialize', 'arm_ready'):
            data += bytes([29, 0, 255, 2])
            for axis in range(3):
                data += struct.pack('<4BI2BH', 2, 0, 15, axis + 1, 5, 13, 4, 0)
            data += bytes([1]) + bytes(15)
        response.data = list(raw if raw is not None else data)
        response.message = 'mock application result'
        return response
    service = node.create_service(Call, 'test_rpc/call', rpc, callback_group=group)
    def gate(request, response):
        response.success = request.deadline_steady_ns > time.monotonic_ns()
        response.token = 123
        return response
    gate_service = node.create_service(ArmTargetGate, 'arm_target/gate', gate, callback_group=group)
    subscriptions = [node.create_subscription(Int32MultiArray, 'pump_state', lambda msg: harness.status.append(list(msg.data)), 10)]
    for topic in ('init_state', 'end_state'):
        subscriptions.append(node.create_subscription(Int32MultiArray, topic, lambda msg: harness.legacy.append(list(msg.data)), 10))
    harness.pump = node.create_client(PumpControl, 'set_pump_state')
    harness.ready = node.create_client(Trigger, 'request_initialization')
    try:
        yield harness
    finally:
        for future in pending:
            if not future.done(): future.set_result(True)
        for child in children:
            if child.poll() is None: os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        for timer in timers: node.destroy_timer(timer)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        print((tmp_path / 'apps.log').read_text())


def test_startup_never_sends_initial_output_or_legacy_state(rig):
    rig.launch()
    assert not rig.requests and not rig.status and not rig.legacy


@pytest.mark.parametrize('selected,values,mask', [(1, (1, 1, 1), 62), (2, (1, 1, 1), 61), (3, (-1, -1, -1), 31)])
def test_pump_waits_for_applied_mask_and_publishes_only_one_confirmation(rig, selected, values, mask):
    rig.launch(timeout=1.5, selected=selected)
    rig.delay = 0.15
    future = rig.set(values)
    rig.until(lambda: bool(rig.requests))
    assert not future.done()
    assert bytes(rig.requests[0].data) == struct.pack('<I', mask)
    assert rig.requests[0].endpoint == 'pump_set'
    assert rig.result(future).success
    rig.spin(0.1)
    assert rig.status == [[], [mask]]
    rig.spin(0.2)
    assert rig.status == [[], [mask]]


@pytest.mark.parametrize('outcome,result,mismatch,raw', [
    (1, 0, False, None), (2, 0, False, None), (3, 0, False, None),
    (4, 0, False, None), (5, 0, False, None), (0, 1, False, None),
    (0, 2, False, None), (0, 3, False, None), (0, 0, True, None), (0, 0, False, b'bad'),
])
def test_unconfirmed_pump_is_false_and_invalidates_display(rig, outcome, result, mismatch, raw):
    rig.launch()
    rig.outcome, rig.app_result, rig.mismatch, rig.raw = outcome, result, mismatch, raw
    assert not rig.result(rig.set()).success
    rig.spin()
    assert all(not sample for sample in rig.status)


def test_busy_and_late_response_cannot_complete_a_new_pump_request(rig):
    rig.launch(timeout=0.12)
    rig.delay = 0.25
    first = rig.set()
    rig.until(lambda: len(rig.requests) == 1)
    assert not rig.result(rig.set()).success
    assert not rig.result(first).success
    rig.delay = 0
    assert rig.result(rig.set((0, 0, 0))).success
    rig.spin(0.3)
    assert rig.status[-1] == [63]
    assert len(rig.requests) == 2


@pytest.mark.parametrize('result,flags,outcome,expected', [
    (0, 31, 0, True), (0, 15, 0, False), (1, 31, 0, False),
    (2, 31, 0, False), (0, 31, 2, False), (0, 31, 3, False),
])
def test_readiness_requires_remote_ready_and_all_flags(rig, result, flags, outcome, expected):
    rig.launch()
    rig.app_result, rig.flags, rig.outcome = result, flags, outcome
    response = rig.result(rig.ready.call_async(Trigger.Request()))
    assert response.success is expected
    assert rig.requests[0].endpoint == 'arm_initialize' and not rig.requests[0].data
    assert all(not request.data for request in rig.requests)
    assert sum(request.endpoint == 'arm_initialize' for request in rig.requests) == 1
    assert not rig.legacy


def test_readiness_delayed_more_than_old_one_second_limit_succeeds(rig):
    rig.launch(timeout=3.0)
    rig.delay = 1.1
    assert rig.result(rig.ready.call_async(Trigger.Request())).success


def test_readiness_timeout_is_not_repaired_by_late_callback(rig):
    rig.launch(timeout=0.12)
    rig.delay = 0.3
    assert not rig.result(rig.ready.call_async(Trigger.Request())).success
    rig.delay = 0
    rig.flags = 0
    assert not rig.result(rig.ready.call_async(Trigger.Request())).success
    rig.spin(0.35)
    assert not rig.legacy

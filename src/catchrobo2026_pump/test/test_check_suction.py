import math
import os
from pathlib import Path
import signal
import subprocess
import struct
import tempfile
import time
import uuid

import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Int32MultiArray, MultiArrayDimension

from canrpc_bridge.srv import Call
from catchrobo2026_msgs.srv import CheckSuction, PumpControl


UNKNOWN = -(2 ** 31)


def pump_executable():
    executable = os.environ.get('PUMP_NODE_EXECUTABLE')
    if not executable:
        from ament_index_python.packages import get_package_prefix
        executable = str(Path(get_package_prefix('catchrobo2026_pump')) /
                         'lib/catchrobo2026_pump/pump_controller_node')
    return executable


class PumpHarness:
    def __init__(self, parameters):
        self.namespace = '/pump_test_' + uuid.uuid4().hex[:12]
        command = [pump_executable(), '--ros-args', '-r', '__ns:=' + self.namespace]
        for name, value in parameters.items():
            command += ['-p', f'{name}:={value}']
        self.log = tempfile.TemporaryFile(mode='w+t')
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)
        self.node = rclpy.create_node('pressure_test_client', namespace=self.namespace)
        def acknowledge(request, response):
            response.outcome = 0
            response.data = list(struct.pack('<I', 0) + bytes(request.data))
            return response
        self.rpc = self.node.create_service(Call, 'canrpc/call', acknowledge)
        self.check = self.node.create_client(CheckSuction, 'check_suction')
        self.set_pump = self.node.create_client(PumpControl, 'set_pump_state')
        self.pressure = self.node.create_publisher(
            Int32MultiArray, 'pressure_sensor', qos_profile_sensor_data)
        self.output = []
        self.subscription = self.node.create_subscription(
            Int32MultiArray, 'pump_state',
            lambda msg: self.output.append((time.monotonic(), msg.data[0])) if msg.data else None, 100)
        try:
            assert self.check.wait_for_service(timeout_sec=8.0)
            assert self.set_pump.wait_for_service(timeout_sec=8.0)
            enabled = parameters.get("pressure_enabled", "true") not in (False, "false")
            self.wait_until(lambda: self.pressure.get_subscription_count() == int(enabled))
            self.spin_for(0.15)
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3.0)
        self.node.destroy_node()
        self.log.seek(0)
        output = self.log.read()
        self.log.close()
        if self.process.returncode:
            print(output)

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=max(0.0, min(0.01, deadline - time.monotonic())))

    def wait_until(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert self.process.poll() is None, 'pump node exited'
            assert time.monotonic() < deadline, 'timed out waiting for ROS event'
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def result(self, future, timeout=3.0):
        self.wait_until(future.done, timeout)
        return future.result()

    def begin(self, mask=7, timeout=0.35):
        future = self.check.call_async(CheckSuction.Request(
            collector_mask=mask, timeout_sec=float(timeout)))
        self.spin_for(0.04)
        return future

    def publish(self, values, duration=0.07, offset=0, dimensions=None):
        message = Int32MultiArray(data=values)
        message.layout.data_offset = offset
        message.layout.dim = dimensions or []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.pressure.publish(message)
            self.spin_for(0.01)


@pytest.fixture(scope='module')
def ros_context():
    # Keep all test traffic off physical CAN topics and outside the production domain.
    previous = {name: os.environ.get(name) for name in (
        'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'ROS_AUTOMATIC_DISCOVERY_RANGE')}
    os.environ['ROS_DOMAIN_ID'] = '229'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()
        for name, value in previous.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


@pytest.fixture(scope='module')
def pump(ros_context):
    harness = PumpHarness({
        'pressure_comparison': 'ge', 'max_pending_suction_checks': 2,
        'valve_on_level': 'true',
        'use_sim_time': 'true',
    })
    try:
        yield harness
    finally:
        harness.close()


def test_no_sample_times_out_with_a_stopped_ros_clock(pump):
    started = time.monotonic()
    response = pump.result(pump.begin(timeout=0.2))
    elapsed = time.monotonic() - started
    assert not response.success and response.timed_out
    assert response.suction_mask == 0 and list(response.pressure) == [UNKNOWN] * 3
    assert 0.18 <= elapsed < 1.0


def test_old_sample_does_not_satisfy_new_request(pump):
    pump.publish([100, 101, 102])
    pump.spin_for(0.05)
    response = pump.result(pump.begin(timeout=0.2))
    assert not response.success and response.timed_out
    assert list(response.pressure) == [UNKNOWN] * 3


def test_threshold_boundary_and_all_collectors_in_one_sample(pump):
    future = pump.begin(timeout=0.65)
    pump.publish([96, 96, 95])
    assert not future.done()
    pump.publish([95, 95, 96])
    assert not future.done(), 'success must not accumulate across samples'
    pump.publish([96, 96, 96])
    response = pump.result(future)
    assert response.success and not response.timed_out
    assert response.suction_mask == 7 and list(response.pressure) == [96] * 3


@pytest.mark.parametrize('mask,values', [
    (1, [96, 0, 0]), (2, [0, 96, 0]), (3, [96, 96, 0]),
    (4, [0, 0, 96]), (5, [96, 0, 96]), (6, [0, 96, 96]), (7, [96, 96, 96]),
])
def test_unselected_collectors_do_not_prevent_success(pump, mask, values):
    future = pump.begin(mask=mask)
    pump.publish(values)
    response = pump.result(future)
    assert response.success and not response.timed_out
    assert response.suction_mask == mask and list(response.pressure) == values


def test_timeout_reports_last_sample_without_latching_partial_success(pump):
    future = pump.begin()
    pump.publish([100, 100, 0])
    pump.publish([0, 0, 100])
    response = pump.result(future)
    assert not response.success and response.timed_out
    assert response.suction_mask == 4 and list(response.pressure) == [0, 0, 100]


def test_invalid_frames_are_ignored_and_next_valid_frame_recovers(pump):
    future = pump.begin(timeout=0.4)
    for values in ([], [100], [100, 100], [100, 100, 100, 100]):
        pump.publish(values, duration=0.025)
    pump.publish([100, 100, 100], duration=0.025, offset=1)
    pump.publish([100, 100, 100], duration=0.025, dimensions=[
        MultiArrayDimension(size=2, stride=2)])
    response = pump.result(future)
    assert not response.success and response.timed_out
    assert list(response.pressure) == [UNKNOWN] * 3
    future = pump.begin()
    pump.publish([96] * 3, dimensions=[MultiArrayDimension(size=3, stride=3)])
    assert pump.result(future).success


@pytest.mark.parametrize('mask,timeout', [
    (0, 0.2), (8, 0.2), (255, 0.2), (7, 0.0), (7, -0.1),
    (7, math.nan), (7, math.inf), (7, 86401.0),
])
def test_invalid_requests_are_rejected_without_waiting(pump, mask, timeout):
    response = pump.result(pump.begin(mask=mask, timeout=timeout), timeout=0.5)
    assert not response.success and not response.timed_out
    assert response.suction_mask == 0 and list(response.pressure) == [UNKNOWN] * 3
    assert response.message


def test_concurrent_limit_independent_masks_and_reusable_slots(pump):
    left = pump.begin(mask=1, timeout=0.5)
    right = pump.begin(mask=4, timeout=0.5)
    rejected = pump.result(pump.begin(mask=2, timeout=0.5))
    assert not rejected.success and not rejected.timed_out
    assert 'too many' in rejected.message
    pump.publish([100, 0, 0])
    assert pump.result(left).success and not right.done()
    center = pump.begin(mask=2, timeout=0.5)
    pump.publish([0, 100, 0])
    assert pump.result(center).success and not right.done()
    response = pump.result(right)
    assert not response.success and response.timed_out
    future = pump.begin()
    pump.publish([100] * 3)
    assert pump.result(future).success


def test_monitor_keeps_rpc_setter_running_without_periodic_output(pump):
    baseline = pump.result(pump.set_pump.call_async(
        PumpControl.Request(left=0, center=0, right=0)))
    assert baseline.success
    pump.wait_until(lambda: pump.output[-1][1] == 7)
    start_count = len(pump.output)
    future = pump.begin(timeout=0.5)
    assert not future.done()
    changed = pump.result(pump.set_pump.call_async(
        PumpControl.Request(left=1, center=0, right=-1)), timeout=0.25)
    assert changed.success and not future.done()
    pump.wait_until(lambda: pump.output[-1][1] == 6)
    response = pump.result(future)
    assert not response.success and response.timed_out
    readings = pump.output[start_count:]
    assert len(readings) == 1, 'confirmed state must be event-only, never a periodic CAN command'
    assert readings[-1][1] == 6, 'monitoring must not alter pump outputs'


def test_reverse_comparison_and_pressure_mapping(ros_context):
    pump = PumpHarness({'pressure_comparison': 'le', 'pressure_indices': '[2, 0, 1]'})
    try:
        future = pump.begin(mask=1)
        pump.publish([97, 98, 96])
        response = pump.result(future)
        assert response.success and response.suction_mask == 1
        assert list(response.pressure) == [96, 97, 98]
        future = pump.begin(mask=7)
        pump.publish([96, 96, 96])
        assert pump.result(future).success
    finally:
        pump.close()


def test_unconfigured_comparison_only_rejects_monitoring(ros_context):
    pump = PumpHarness({'pressure_comparison': 'unconfigured'})
    try:
        response = pump.result(pump.begin())
        assert not response.success and not response.timed_out
        assert 'configured' in response.message
        assert pump.result(pump.set_pump.call_async(PumpControl.Request())).success
    finally:
        pump.close()


def test_shutdown_does_not_wait_for_pending_service(ros_context):
    pump = PumpHarness({'pressure_comparison': 'ge'})
    try:
        future = pump.begin(timeout=86400.0)
        assert not future.done()
        pump.process.send_signal(signal.SIGINT)
        assert pump.process.wait(timeout=2.0) == 0
    finally:
        pump.close()


@pytest.mark.parametrize('name,value', [
    ('pressure_indices', '[0, 1]'), ('pressure_indices', '[0, 0, 2]'),
    ('pressure_indices', '[0, 1, 3]'), ('pressure_indices', '[-1, 1, 2]'),
    ('pressure_threshold', '2147483648'), ('pressure_threshold', '-2147483649'),
    ('pressure_comparison', 'invalid'), ('max_pending_suction_checks', '0'),
    ('max_pending_suction_checks', '129'),
])
def test_invalid_startup_configuration_is_rejected(ros_context, tmp_path, name, value):
    process = subprocess.run([
        pump_executable(), '--ros-args', '-r', '__ns:=/pump_invalid_' + uuid.uuid4().hex[:12],
        '-p', f'{name}:={value}',
    ], cwd=tmp_path, capture_output=True, text=True, timeout=4.0)
    assert process.returncode == 1
    assert name in process.stderr


@pytest.mark.parametrize('selected', [1, 2, 3])
def test_only_selected_collector_can_be_active_on_direct_service(ros_context, selected):
    harness = PumpHarness({'selected_pump': selected})
    try:
        assert not harness.output
        assert harness.result(harness.set_pump.call_async(
            PumpControl.Request(left=1, center=1, right=1))).success
        expected = 63 & ~(1 << (selected - 1))
        harness.wait_until(lambda: harness.output[-1][1] == expected)
        assert all(value == expected for _, value in harness.output)
        assert harness.result(harness.set_pump.call_async(
            PumpControl.Request(left=-1, center=-1, right=-1))).success
        expected = 63 & ~(1 << (selected + 2))
        harness.wait_until(lambda: harness.output[-1][1] == expected)
        harness.spin_for(0.05)
        assert harness.output[-1][1] == expected
        assert harness.result(harness.set_pump.call_async(PumpControl.Request())).success
        harness.wait_until(lambda: harness.output[-1][1] == 63)
    finally:
        harness.close()


def test_disabled_pressure_is_immediate_unknown_and_pump_control_continues(ros_context):
    harness = PumpHarness({'pressure_enabled': 'false', 'pressure_comparison': 'ge'})
    try:
        harness.publish([2147483647] * 3)
        started = time.monotonic()
        response = harness.result(harness.begin(timeout=86400.0), timeout=1.0)
        assert time.monotonic() - started < 0.5
        assert not response.success and not response.timed_out
        assert response.suction_mask == 0 and list(response.pressure) == [UNKNOWN] * 3
        assert 'disabled' in response.message
        assert harness.pressure.get_subscription_count() == 0
        parameters = harness.node.create_client(SetParameters, 'pump_controller_node/set_parameters')
        assert parameters.wait_for_service(timeout_sec=2.0)
        changed = harness.result(parameters.call_async(SetParameters.Request(parameters=[
            Parameter(name='pressure_enabled', value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=True))])))
        assert not changed.results[0].successful
        assert harness.pressure.get_subscription_count() == 0
        assert harness.result(harness.set_pump.call_async(
            PumpControl.Request(left=1, center=1, right=1))).success
        harness.wait_until(lambda: bool(harness.output))
        assert harness.result(harness.set_pump.call_async(PumpControl.Request())).success
    finally:
        harness.close()


def test_reenabled_pressure_uses_fresh_samples_after_restart(ros_context):
    harness = PumpHarness({'pressure_enabled': 'true', 'pressure_comparison': 'ge'})
    try:
        future = harness.begin(timeout=1.0)
        assert not future.done()
        harness.publish([100, 101, 102])
        response = harness.result(future)
        assert response.success and not response.timed_out and response.suction_mask == 7
        assert list(response.pressure) == [100, 101, 102]
    finally:
        harness.close()

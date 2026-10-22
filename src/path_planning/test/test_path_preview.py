"""Verify the path-only RViz marker publisher without a display server."""
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
import yaml


def flange_position(q):
    radius = 0.480 * (math.sin(q[1]) + math.sin(q[2]))
    return (0.675 + radius * math.sin(q[0]),
            -0.190 + radius * math.cos(q[0]),
            0.090 + 0.480 * (math.cos(q[1]) + math.cos(q[2])))


def main():
    executable, urdf_path = sys.argv[1:]
    os.environ['ROS_DOMAIN_ID'] = '91'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    with tempfile.TemporaryDirectory(prefix='path_preview_test_') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        params = Path(directory) / 'params.yaml'
        params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
            'robot_description': Path(urdf_path).read_text(),
        }}}))
        rclpy.init()
        node = rclpy.create_node('preview_test')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(JointTrajectory, '/geometric_path', qos)
        process = subprocess.Popen([executable, '--ros-args', '--params-file', str(params),
                                    '-r', '__ns:=/path_preview',
                                    '-r', 'geometric_path:=/geometric_path'])
        try:
            markers = []
            subscription = node.create_subscription(Marker, '/path_preview/path_marker', markers.append, qos)
            deadline = time.monotonic() + 10
            while not publisher.get_subscription_count():
                assert time.monotonic() < deadline, 'Discovery timeout'
                rclpy.spin_once(node, timeout_sec=0.1)

            def send(message, action):
                markers.clear()
                publisher.publish(message)
                deadline = time.monotonic() + 10
                while not markers or markers[-1].action != action:
                    assert time.monotonic() < deadline, 'Marker timeout'
                    rclpy.spin_once(node, timeout_sec=0.1)
                return markers[-1]

            a, b = [0., 0., 1.5, 0.], [0.4, 0.2, 1.2, 0.3]
            message = JointTrajectory()
            message.joint_names = ['theta4', 'theta2', 'theta1', 'theta3']
            for q in [a, b]:
                point = JointTrajectoryPoint()
                point.positions = [q[3], q[1], q[0], q[2]]
                message.points.append(point)
            marker = send(message, Marker.ADD)
            assert marker.type == Marker.LINE_STRIP and len(marker.points) > 2
            assert marker.header.frame_id == 'map'
            for point, q in [(marker.points[0], a), (marker.points[-1], b)]:
                assert all(abs(actual - expected) < 1e-10 for actual, expected in
                           zip((point.x, point.y, point.z), flange_position(q)))
            # A subscriber that joins after planning still receives the path.
            late = []
            late_subscription = node.create_subscription(Marker, '/path_preview/path_marker', late.append, qos)
            deadline = time.monotonic() + 5
            while not late:
                assert time.monotonic() < deadline
                rclpy.spin_once(node, timeout_sec=0.1)
            assert late[-1].action == Marker.ADD
            node.destroy_subscription(late_subscription)
            assert not send(JointTrajectory(), Marker.DELETE).points
            message.points = message.points[:1]
            assert send(message, Marker.ADD).type == Marker.SPHERE_LIST
            message.points[0].positions[0] = float('nan')
            send(message, Marker.DELETE)
            assert not node.get_publishers_info_by_topic('/path_preview/preview_joint_states')
            print('Path marker coordinates, interpolation, late subscription and clearing passed')
        finally:
            process.terminate()
            process.wait(timeout=10)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

"""ROS integration smoke test; takes executable and expanded URDF paths."""
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
import yaml


def main():
    executable, urdf_path = sys.argv[1:]
    # Isolated from the normal robot domain; all endpoints are test processes.
    os.environ['ROS_DOMAIN_ID'] = '89'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    with tempfile.TemporaryDirectory(prefix='arm_collision_ros_') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        params = Path(directory) / 'params.yaml'
        params.write_text(yaml.safe_dump({'path_planner': {'ros__parameters': {
            'robot_description': Path(urdf_path).read_text(),
        }}}))
        rclpy.init()
        node = rclpy.create_node('arm_collision_smoke')
        process = subprocess.Popen([executable, '--ros-args', '--params-file', str(params)])
        try:
            start = node.create_publisher(JointState, 'start_joint_states', 10)
            goal = node.create_publisher(JointState, 'goal_joint_states', 10)
            paths = []
            subscription = node.create_subscription(JointTrajectory, 'geometric_path', paths.append, 10)
            client = node.create_client(Trigger, 'plan_path')
            assert client.wait_for_service(timeout_sec=10), 'Planner service did not start'
            deadline = time.monotonic() + 10
            while not start.get_subscription_count() or not goal.get_subscription_count():
                assert time.monotonic() < deadline, 'Topic discovery timed out'
                rclpy.spin_once(node, timeout_sec=0.1)

            def request(expected):
                count = len(paths)
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=10)
                assert future.done() and future.result().success == expected
                deadline = time.monotonic() + 5
                while len(paths) <= count:
                    assert time.monotonic() < deadline, 'Path publish timed out'
                    rclpy.spin_once(node, timeout_sec=0.1)
                return paths[-1]

            # Missing inputs fail closed.
            assert not request(False).points
            a = JointState()
            a.name = ['theta4', 'theta2', 'theta1', 'theta3']
            a.position = [0.0, 0.0, 0.0, 1.5]
            b = JointState()
            b.position = [0.2, 0.2, 1.5, 0.0]
            start.publish(a)
            goal.publish(b)
            time.sleep(0.3)
            path = request(True)
            assert list(path.points[0].positions) == [0.0, 0.0, 1.5, 0.0]
            assert list(path.points[-1].positions) == list(b.position)

            # Valid-sized but physically colliding goal is rejected and clears old path.
            b.position = [0.0, math.pi / 2, -math.pi / 2, 0.0]
            goal.publish(b)
            time.sleep(0.3)
            assert not request(False).points

            # An overlap between forearm and hand must now be accepted.
            a.position = [0.0, 0.0, 0.0, 0.0]
            b.position = [0.0, 0.0, 0.0, 0.0]
            start.publish(a)
            goal.publish(b)
            time.sleep(0.3)
            assert request(True).points

            # Restore a valid goal to confirm that invalidation is recoverable.
            b.position = [0.2, 0.2, 1.5, 0.0]
            goal.publish(b)
            time.sleep(0.3)
            assert request(True).points
            print('ROS collision rejection, recovery, joint ordering and path clearing passed')
        finally:
            process.terminate()
            process.wait(timeout=10)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

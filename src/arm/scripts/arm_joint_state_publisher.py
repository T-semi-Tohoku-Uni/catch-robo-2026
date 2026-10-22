#!/usr/bin/env python3
"""Convert the IK's four absolute angles into the URDF serial chain angles."""
import math
import signal

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState

INPUT_NAMES = ['theta1', 'theta2', 'theta3', 'theta4']
JOINT_NAMES = [
    'base_yaw_joint', 'shoulder_joint', 'elbow_joint',
    'wrist_level_joint', 'flange_yaw_joint',
]


def urdf_positions(angles):
    if len(angles) != 4 or not all(math.isfinite(a) for a in angles):
        raise ValueError('Expected four finite angles in radians')
    theta1, theta2, theta3, theta4 = angles
    return [theta1, theta2, theta3 - theta2, -theta3,
            2.0 * theta1 + theta4 - math.pi / 2.0]


class ArmJointStatePublisher(Node):
    def __init__(self):
        super().__init__('arm_joint_state_publisher')
        angles = self.declare_parameter('initial_angles', [0.0, 0.0, 0.0, 0.0]).value
        self.positions = urdf_positions(angles)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(
            JointState, 'arm/absolute_joint_states', self.update_angles, 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_state)

    def update_angles(self, message):
        try:
            if message.name:
                if len(message.name) != len(message.position):
                    raise ValueError('Joint names and positions must have equal length')
                if len(set(message.name)) != len(message.name):
                    raise ValueError('Duplicate joint names')
                positions = dict(zip(message.name, message.position))
                angles = [positions[name] for name in INPUT_NAMES]
            else:
                angles = list(message.position)
            self.positions = urdf_positions(angles)
        except (ValueError, KeyError) as error:
            self.get_logger().warning(f'Ignoring invalid IK joint state: {error}')

    def publish_state(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = JOINT_NAMES
        message.position = self.positions
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ArmJointStatePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch can forward SIGINT after the terminal already sent it.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

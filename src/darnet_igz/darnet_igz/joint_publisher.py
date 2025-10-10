#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        self.count = 0
        # 100 ms
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Add names of the joints you want to control
        msg.joint_names = [
            'Kepala Putar',
            'Leher Putar',
            'Paha Atas Kanan',
            'Paha Atas Kiri',
        ]

        pos1 = 1.5 * (1.0 - math.cos(self.count * 0.1))
        pos2 = -pos1

        point = JointTrajectoryPoint()
        point.positions = [pos1, pos2]
        point.time_from_start = Duration(seconds=1.0).to_msg()

        msg.points = [point]
        self.pub.publish(msg)
        self.get_logger().info(f"{msg}")

        self.get_logger().info(f'Publishing: {pos1:.3f}, {pos2:.3f}')
        self.count += 1


def main():
    rclpy.init()
    node = JointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

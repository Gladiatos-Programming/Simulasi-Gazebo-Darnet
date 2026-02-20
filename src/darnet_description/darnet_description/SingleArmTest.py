#!/usr/bin/env python3
import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SingleArmTestController(Node):
    def __init__(self):
        super().__init__('single_arm_test_controller')

        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Pola referensi dari tangan kanan.
        self.base = [-0.05, -1.25, 0.75]
        self.amp = [0.04, -0.35, 0.05]

        # Gerakkan dua tangan dengan pola yang sama.
        self.arm_joints = [
            'Bahu Tangan Kiri', 'Lengan Kiri', 'Tangan Kiri',
            'Bahu Tangan Kanan', 'Lengan Kanan', 'Tangan Kanan'
        ]

        self.speed = 0.4
        self.t = 0.0
        self.dt = 0.08
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info('Dual arm synced test started')
        self.get_logger().info(f'Joints: {self.arm_joints}')

    def timer_callback(self):
        self.t += self.dt
        phase = self.speed * self.t

        pos_0 = self.base[0] + self.amp[0] * math.sin(phase)
        pos_1 = self.base[1] + self.amp[1] * math.sin(phase + math.pi / 3.0)
        pos_2 = self.base[2] + self.amp[2] * math.sin(phase + math.pi / 2.0)

        msg = JointTrajectory()
        msg.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        point.positions = [
            pos_0, pos_1, pos_2,  # Left arm
            pos_0, pos_1, pos_2   # Right arm (same pattern)
        ]
        point.time_from_start = Duration(sec=0, nanosec=500000000)
        msg.points.append(point)

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SingleArmTestController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

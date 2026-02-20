#!/usr/bin/env python3
import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArcheryArmsOnly(Node):
    def __init__(self):
        super().__init__('single_arm_test_controller')

        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Gerakkan dua tangan sekaligus.
        self.arm_joints = [
            'Bahu Tangan Kiri', 'Lengan Kiri', 'Tangan Kiri',
            'Bahu Tangan Kanan', 'Lengan Kanan', 'Tangan Kanan'
        ]

        # Nilai kanan harus mirror (kebalikan tanda) dari kiri
        # supaya bentuk gerak terlihat sama secara visual.
        self.base_left = [0.05, 1.25, 0.75]
        self.amp_left = [0.04, -0.35, 0.05]
        self.base_right = [-0.05, -1.25, -0.75]
        self.amp_right = [-0.04, 0.35, -0.05]

        self.speed = 0.22
        self.t = 0.0
        self.dt = 0.08
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info('ArcheryArmsOnly started: both arms active')
        self.get_logger().info(f'Joints: {self.arm_joints}')

    def timer_callback(self):
        self.t += self.dt
        phase = self.speed * self.t

        draw = 0.5 * (1.0 + math.sin(phase))  # 0..1
        grip = 0.5 * (1.0 + math.sin(phase + math.pi / 2.0))  # 0..1

        # Left arm: hold bow (mostly stable)
        left_0 = self.base_left[0] + 0.01 * (draw - 0.5)
        left_1 = self.base_left[1] + 0.06 * (grip - 0.5)
        left_2 = self.base_left[2] + 0.04 * (grip - 0.5)

        # Right arm: draw bow string (main motion)
        right_0 = self.base_right[0] - 0.10 * draw
        right_1 = self.base_right[1] + 0.45 * draw
        right_2 = self.base_right[2] - 0.22 * grip

        msg = JointTrajectory()
        msg.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        point.positions = [left_0, left_1, left_2, right_0, right_1, right_2]
        point.time_from_start = Duration(sec=0, nanosec=650000000)
        msg.points.append(point)

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArcheryArmsOnly()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

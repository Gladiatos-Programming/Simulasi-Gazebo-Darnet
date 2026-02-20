#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_srvs.srv import Trigger


class ArcheryPoseController(Node):
    def __init__(self):
        super().__init__('archery_pose_controller')

        # Topic IK untuk kaki
        self.topic_right_leg = '/target_pose/right_leg'
        self.topic_left_leg = '/target_pose/left_leg'
        self.topic_right_arm = '/target_pose/right_arm'
        self.topic_left_arm = '/target_pose/left_arm'

        self.pub_right_leg = self.create_publisher(Pose, self.topic_right_leg, 10)
        self.pub_left_leg = self.create_publisher(Pose, self.topic_left_leg, 10)
        self.pub_right_arm = self.create_publisher(Pose, self.topic_right_arm, 10)
        self.pub_left_arm = self.create_publisher(Pose, self.topic_left_arm, 10)

        self.default_quat = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}

        # Konfigurasi pose jongkok + memanah
        # Gunakan base kaki yang mirip script jalan agar tidak silang.
        self.right_leg_base = {'x': 0.21, 'y': -0.04, 'z': 0.01}
        self.left_leg_base = {'x': 0.045, 'y': -0.04, 'z': 0.01}
        self.crouch_depth = 0.030
        self.breath_amp = 0.003

        self.leg_speed = 1.2
        self.arm_speed = 0.22

        self.t = 0.0
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.reset_client = self.create_client(Trigger, '/reset_ik_memory')
        self.reset_ik_node()
        time.sleep(0.5)

        self.get_logger().info('🏹 Archery pose node started')

    def timer_callback(self):
        self.t += self.dt
        phase_leg = self.leg_speed * self.t
        phase_arm = self.arm_speed * self.t

        # Gerakan halus seperti nafas saat jongkok
        breath = math.sin(phase_leg) * self.breath_amp

        # Kaki: jongkok stabil
        right_leg = Pose()
        right_leg.position.x = self.right_leg_base['x']
        right_leg.position.y = self.right_leg_base['y']
        right_leg.position.z = self.right_leg_base['z'] + self.crouch_depth + breath
        right_leg.orientation = self.quat_msg(self.default_quat)

        left_leg = Pose()
        left_leg.position.x = self.left_leg_base['x']
        left_leg.position.y = self.left_leg_base['y']
        left_leg.position.z = self.left_leg_base['z'] + self.crouch_depth + breath
        left_leg.orientation = self.quat_msg(self.default_quat)

        self.pub_right_leg.publish(right_leg)
        self.pub_left_leg.publish(left_leg)

        # ===== TANGAN MEMANAH (IK target pose, pola mirror) =====
        base_left = [0.08, 0.18, 0.19]
        amp_left = [0.005, 0.020, 0.010]
        base_right = [-0.08, 0.18, 0.19]
        amp_right = [-0.005, -0.020, -0.010]

        left_arm = Pose()
        left_arm.position.x = base_left[0] + amp_left[0] * math.sin(phase_arm)
        left_arm.position.y = base_left[1] + amp_left[1] * math.sin(phase_arm + math.pi / 3.0)
        left_arm.position.z = base_left[2] + amp_left[2] * math.sin(phase_arm + math.pi / 2.0)
        left_arm.orientation = self.quat_msg(self.default_quat)

        right_arm = Pose()
        right_arm.position.x = base_right[0] + amp_right[0] * math.sin(phase_arm)
        right_arm.position.y = base_right[1] + amp_right[1] * math.sin(phase_arm + math.pi / 3.0)
        right_arm.position.z = base_right[2] + amp_right[2] * math.sin(phase_arm + math.pi / 2.0)
        right_arm.orientation = self.quat_msg(self.default_quat)

        self.pub_left_arm.publish(left_arm)
        self.pub_right_arm.publish(right_arm)

    def quat_msg(self, q_dict):
        o = Pose().orientation
        o.x = q_dict['x']
        o.y = q_dict['y']
        o.z = q_dict['z']
        o.w = q_dict['w']
        return o

    def reset_ik_node(self):
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Reset IK service tidak tersedia')
            return
        req = Trigger.Request()
        self.reset_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = ArcheryPoseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

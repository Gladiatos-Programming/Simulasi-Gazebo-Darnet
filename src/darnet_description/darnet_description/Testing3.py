#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import Int64
from std_srvs.srv import Trigger


class DualLegWalkerBackward(Node):
    """Dual-leg gait with runtime direction control.

    Direction values:
    - 1  : forward walk
    - -1 : backward walk
    - 0  : walk in place (no forward translation)
    """

    def __init__(self):
        super().__init__('dual_leg_walker_backward')

        # ================= CONFIGURATION =================
        self.speed = 5.0
        self.step_height = 0.10
        self.walking_state = 0
        self.walk_direction = -1

        self.topic_right = '/target_pose/right_leg'
        self.base_right = {'x': 0.21, 'y': -0.04, 'z': 0.01}

        self.topic_left = '/target_pose/left_leg'
        self.base_left = {'x': 0.045, 'y': -0.04, 'z': 0.01}

        self.default_quat = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}

        # Gait gains
        # IMPORTANT: pada robot ini maju-mundur lebih cocok di sumbu Y,
        # sedangkan X dipakai untuk jarak kiri-kanan kaki.
        self.forward_amplitude = 0.035
        self.lateral_amplitude = 0.006
        self.hip_pitch_sign = 1.0  # ubah ke -1 bila arah paha masih terbalik

        self.x_bias_right = -0.02
        self.x_bias_left = 0.02
        self.z_lift_offset = 0.13
        self.z_scale = 0.08

        self.pub_right = self.create_publisher(Pose, self.topic_right, 10)
        self.pub_left = self.create_publisher(Pose, self.topic_left, 10)

        self.timer = self.create_timer(0.02, self.timer_callback)
        self.t = 0.0
        self.dt = 0.02

        self.speed_sub = self.create_subscription(
            Int64, '/speed_step', self.callback_change_speed_step, 10
        )
        self.height_sub = self.create_subscription(
            Int64, '/step_height', self.callback_change_step_height, 10
        )
        self.start_sub = self.create_subscription(
            Int64, '/start_walking', self.callback_start_walking, 10
        )
        self.direction_sub = self.create_subscription(
            Int64, '/walk_direction', self.callback_walk_direction, 10
        )

        self.reset_client = self.create_client(Trigger, '/reset_ik_memory')
        self.reset_ik_node()

        self.get_logger().info('🚶 DUAL LEG WALKER (TESTING3) STARTED')
        self.get_logger().info('   Default mode: backward walk (walk_direction=-1)')
        time.sleep(0.5)

    def callback_start_walking(self, msg):
        self.walking_state = msg.data
        if self.walking_state == 1:
            self.get_logger().info('Starting Walking...')
        else:
            self.get_logger().info('Stopping Walking...')

    def callback_change_speed_step(self, msg):
        self.speed = float(msg.data)
        self.get_logger().info(f'Step Speed Updated to: {self.speed}')

    def callback_change_step_height(self, msg):
        self.step_height = float(msg.data) * 0.01
        self.get_logger().info(f'Step Height Updated to: {self.step_height}')

    def callback_walk_direction(self, msg):
        requested = int(msg.data)
        if requested > 0:
            self.walk_direction = 1
        elif requested < 0:
            self.walk_direction = -1
        else:
            self.walk_direction = 0

        direction_name = {
            1: 'forward',
            -1: 'backward',
            0: 'in-place',
        }[self.walk_direction]
        self.get_logger().info(f'Walk direction changed to: {direction_name}')

    def _leg_offsets(self, elapsed, phase):
        gait_phase = self.speed * elapsed + phase - 0.5

        z_raw = math.sin(gait_phase) * (self.step_height + self.z_scale) - self.z_lift_offset
        z = max(0.0, z_raw)

        # Maju-mundur sekarang dipindah ke sumbu Y agar tidak geser kanan-kiri.
        y_forward = (
            self.forward_amplitude
            * math.sin(gait_phase)
            * self.walk_direction
            * self.hip_pitch_sign
        )

        # X hanya untuk sway kecil supaya kaki tetap stabil kiri-kanan.
        x_lateral = math.sin(gait_phase - 1.57) * self.lateral_amplitude
        return x_lateral, y_forward, z

    def timer_callback(self):
        self.t += self.dt
        elapsed = self.t

        x_r_lateral, y_r_forward, z_right = self._leg_offsets(elapsed, 0.0)
        x_l_lateral, y_l_forward, z_left = self._leg_offsets(elapsed, math.pi)

        x_right = max(-0.05, min(x_r_lateral + self.x_bias_right, 0.02))
        x_left = max(-0.02, min(x_l_lateral + self.x_bias_left, 0.05))

        msg_r = Pose()
        msg_r.position.x = self.base_right['x'] + x_right
        msg_r.position.y = (
            self.base_right['y'] + y_r_forward if self.walking_state == 1 else self.base_right['y']
        )
        msg_r.position.z = self.base_right['z'] + z_right
        msg_r.orientation.w = self.default_quat['w']
        msg_r.orientation.x = self.default_quat['x']
        msg_r.orientation.y = self.default_quat['y']
        msg_r.orientation.z = self.default_quat['z']
        self.pub_right.publish(msg_r)

        msg_l = Pose()
        msg_l.position.x = self.base_left['x'] + x_left
        msg_l.position.y = (
            self.base_left['y'] + y_l_forward if self.walking_state == 1 else self.base_left['y']
        )
        msg_l.position.z = self.base_left['z'] + z_left
        msg_l.orientation.w = self.default_quat['w']
        msg_l.orientation.x = self.default_quat['x']
        msg_l.orientation.y = self.default_quat['y']
        msg_l.orientation.z = self.default_quat['z']
        self.pub_left.publish(msg_l)

    def reset_ik_node(self):
        self.get_logger().info('🔄 Requesting IK Memory Reset...')
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('❌ Reset Service not available!')
            return
        req = Trigger.Request()
        self.reset_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = DualLegWalkerBackward()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

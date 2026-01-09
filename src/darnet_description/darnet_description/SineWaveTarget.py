#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
import time

class DualLegWalker(Node):
    def __init__(self):
        super().__init__('dual_leg_walker')
        
        # ================= CONFIGURATION =================
        self.speed = 10.0         # Kecepatan Jalan
        self.step_height = 0.05  # Tinggi angkat kaki (5 cm)
        
        # --- KAKI KANAN (RIGHT) ---
        self.topic_right = '/target_pose/right_leg'
        self.base_right = {'x': 0.2, 'y': -0.04, 'z': 0.01}
        
        # --- KAKI KIRI (LEFT) ---
        self.topic_left = '/target_pose/left_leg'
        self.base_left = {'x': 0.06, 'y': -0.06, 'z': 0.01} 
        
        # Orientasi (Default Lurus)
        self.default_quat = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        # =================================================
        
        # Publishers
        self.pub_right = self.create_publisher(Pose, self.topic_right, 10)
        self.pub_left = self.create_publisher(Pose, self.topic_left, 10)
        
        # Timer (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.start_time = time.time()
        
        self.get_logger().info("🚶 DUAL LEG WALKER STARTED")
        self.get_logger().info(f"   Speed: {self.speed} | Height: {self.step_height}m")

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        
        # === HITUNG MATEMATIKA ===
        
        # 1. Kaki Kanan (Fase Normal) -> (1 - cos(t))
        # Mulai dari 0 -> Naik -> Turun
        z_right = (self.step_height / 2.0) * (1 - math.cos(self.speed * elapsed))
        
        # 2. Kaki Kiri (Fase Berlawanan) -> (1 - cos(t + PI))
        # Ditambah PI (3.14) agar gelombangnya geser 180 derajat
        z_left = (self.step_height / 2.0) * (1 - math.cos(self.speed * elapsed + math.pi))
        
        # === PUBLISH KANAN ===
        msg_r = Pose()
        msg_r.position.x = self.base_right['x']
        msg_r.position.y = self.base_right['y']
        msg_r.position.z = self.base_right['z'] + z_right
        msg_r.orientation.w = self.default_quat['w']
        self.pub_right.publish(msg_r)
        
        # === PUBLISH KIRI ===
        msg_l = Pose()
        msg_l.position.x = self.base_left['x']
        msg_l.position.y = self.base_left['y']
        msg_l.position.z = self.base_left['z'] + z_left
        msg_l.orientation.w = self.default_quat['w']
        self.pub_left.publish(msg_l)

def main(args=None):
    rclpy.init(args=args)
    node = DualLegWalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
import time
from std_srvs.srv import Trigger
from std_msgs.msg import Int64

class DualLegWalker(Node):
    def __init__(self):
        super().__init__('dual_leg_walker')
        
        # ================= CONFIGURATION =================
        self.speed = 8.0         # Kecepatan Jalan
        self.step_height = 0.03  # Tinggi angkat kaki (5 cm)
        
        # --- KAKI KANAN (RIGHT) ---
        self.topic_right = '/target_pose/right_leg'
        self.base_right = {'x': 0.21, 'y': -0.04, 'z': 0.01}
        
        # --- KAKI KIRI (LEFT) ---
        self.topic_left = '/target_pose/left_leg'
        self.base_left = {'x': 0.045, 'y': -0.04, 'z': 0.01}
        
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

        self.speed_pub = self.create_publisher(Int64, '/servo_speed_ns', 10)

        self.reset_client = self.create_client(Trigger, '/reset_ik_memory')
        self.reset_ik_node()

        # Set IK Speed
        # self.set_ik_speed(5000000)

        time.sleep(0.5)

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
        msg_r.orientation.x = self.default_quat['x']
        msg_r.orientation.y = self.default_quat['y']
        msg_r.orientation.z = self.default_quat['z']
        self.pub_right.publish(msg_r)
        
        # === PUBLISH KIRI ===
        msg_l = Pose()
        msg_l.position.x = self.base_left['x']
        msg_l.position.y = self.base_left['y']
        msg_l.position.z = self.base_left['z'] + z_left
        msg_l.orientation.w = self.default_quat['w']
        msg_l.orientation.x = self.default_quat['x']
        msg_l.orientation.y = self.default_quat['y']
        msg_l.orientation.z = self.default_quat['z']
        self.pub_left.publish(msg_l)

    def reset_ik_node(self):
        """Fungsi untuk menekan tombol reset di Node IK"""
        self.get_logger().info("🔄 Requesting IK Memory Reset...")
        
        # Tunggu service tersedia
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Reset Service not available!")
            return

        # Tekan tombolnya
        req = Trigger.Request()
        future = self.reset_client.call_async(req)

    def set_ik_speed(self, nanosecond):
        """Helper function untuk set speed dalam detik"""
        msg = Int64()
        # Detik ke Nanosekon (1 detik = 1 milyar ns)
        msg.data = int(nanosecond)
        self.speed_pub.publish(msg)
        self.get_logger().info(f"🐢 Setting IK Speed to {nanosecond} nanoseconds...")
        # Kasih jeda dikit biar message speed sampai duluan sebelum target pose
        time.sleep(0.1)

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
    
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
        self.speed = 2.0        # Kecepatan Jalan
        self.step_height = 0.09  # Tinggi angkat kaki (5 cm)
        
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
        # self.start_time = time.time()
        self.t = 0.0  # Waktu internal robot
        self.dt = 0.02 # Kenaikan waktu per loop (sesuai timer 50Hz)
        
        self.get_logger().info("🚶 DUAL LEG WALKER STARTED")
        self.get_logger().info(f"   Speed: {self.speed} | Height: {self.step_height}m")

        # Subscription
        self.speed_pub = self.create_subscription(Int64, '/speed_step', self.callback_change_speed_step, 10)
        self.step_height_pub = self.create_subscription(Int64, '/step_height', self.callback_change_step_height, 10)

        self.reset_client = self.create_client(Trigger, '/reset_ik_memory')
        self.reset_ik_node()

        # Set IK Speed
        # self.set_ik_speed(5000000)

        time.sleep(0.5)

    def callback_change_speed_step(self, msg):
        """Callback untuk ubah speed dan step height dari luar"""
        self.speed = msg.data
        self.get_logger().info(f"Step Speed Updated to: {self.speed}")

    def callback_change_step_height(self, msg):
        """Callback untuk ubah step height dari luar"""
        self.step_height = msg.data * 0.01
        self.get_logger().info(f"Step Height Updated to: {self.step_height}")

    def timer_callback(self):
        # elapsed = time.time() - self.start_time
        self.t += self.dt
        elapsed = self.t

        self.step_height2 = 0.1

        
        # === HITUNG MATEMATIKA ===
        
        # 1. Kaki Kanan (Fase 0)
        # Sinyal Sinus biasa. Kalau positif dia naik, kalau negatif dia 0 (napak tanah)
        # raw_sin_r = (math.sin(self.speed * elapsed) * self.step_height)
        # z_right = max(0, raw_sin_r)
        
        # # 2. Kaki Kiri (Fase 180 derajat / PI)
        # # Geser gelombang biar gantian. Saat kanan napak, kiri ngangkat.
        # raw_sin_l = (math.sin(self.speed * elapsed + math.pi) * self.step_height)
        # z_left = max(0, raw_sin_l)

        # # 3. Pergerakan (X axis)
        # raw_x_right = 0.04 * math.sin(self.speed * elapsed) * 1.7
        # x_right = max(-0.04, min(raw_x_right, 0.04))

        # raw_x_left = -0.04 * math.sin(self.speed * elapsed + math.pi) * 1.7
        # x_left = max(-0.04, min(raw_x_left, 0.04))

        # # 4. Pergerakkan Y nya
        # raw_y_right1 = math.sin(self.speed/2 * elapsed + 0.319) - 0.95
        # raw_y_right2 = math.sin(self.speed/2 * elapsed + 0.319 + 3.14) - 0.95
        # y_right = max(-0.05, raw_y_right1, raw_y_right2)

        # raw_y_left1 = math.sin(self.speed/2 * elapsed - 1.25) - 0.95
        # raw_y_left2 = math.sin(self.speed/2 * elapsed - 1.25 + 3.14) - 0.95
        # y_left = max(-0.05, raw_y_left1, raw_y_left2)

        # === HITUNG MATEMATIKA V2 ===
        raw_z_right = math.sin(self.speed * elapsed - 0.6) * (self.step_height2 + 0.08) - 0.13
        z_right = max(0, raw_z_right)

        raw_z_left = math.sin(self.speed * elapsed + math.pi - 0.6) * (self.step_height2 + 0.08) - 0.13
        z_left = max(0, raw_z_left)

        raw_x_right = 0.02 * math.sin(self.speed * elapsed - 0.6) * 3 - 0.02
        x_right = max(-0.05,min(raw_x_right,0.01))

        raw_x_left = 0.02 * math.sin(self.speed * elapsed - 0.6) * 3 + 0.02
        x_left = max(-0.01,min(raw_x_left,0.05))

        # raw_y_right = math.sin(self.speed/2 * elapsed + 0.39) * (self.step_height2 + 0.08) - 0.13
        # raw_y_right2 = math.sin(self.speed/2 * elapsed + 0.39 + 3.14) * (self.step_height2 + 0.08) - 0.13

        y_right = math.sin(self.speed * elapsed - 1.57 - 0.6) * 0.025
        # y_right = max(0.0, raw_y_right)

        # raw_y_left = math.sin(self.speed/2 * elapsed + 0.39 + 1.85) * (self.step_height2 + 0.08) - 0.13
        # raw_y_left2 = math.sin(self.speed/2 * elapsed + 0.39 + 1.85 + 3.14) * (self.step_height2 + 0.08) - 0.13 
        # y_left = max(0.0, raw_y_left, raw_y_left2)
        y_left = math.sin(self.speed * elapsed - 1.57 + 3.14 - 0.6) * 0.025

        # === PUBLISH KANAN ===
        msg_r = Pose()
        msg_r.position.x = self.base_right['x'] + x_right
        msg_r.position.y = self.base_right['y'] + y_right
        msg_r.position.z = self.base_right['z'] + z_right
        msg_r.orientation.w = self.default_quat['w']
        msg_r.orientation.x = self.default_quat['x']
        msg_r.orientation.y = self.default_quat['y']
        msg_r.orientation.z = self.default_quat['z']
        self.pub_right.publish(msg_r)
        
        # === PUBLISH KIRI ===
        msg_l = Pose()
        msg_l.position.x = self.base_left['x'] + x_left
        msg_l.position.y = self.base_left['y'] + y_left  
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
    
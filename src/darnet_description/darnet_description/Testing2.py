#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
import time

class WalkingPatternGenerator(Node):
    def __init__(self):
        super().__init__('walking_pattern_generator')
        
        # Publisher ke topik yang dibaca oleh script Differential IK
        self.pub_right = self.create_publisher(Pose, '/target_pose/right_leg', 10)
        self.pub_left = self.create_publisher(Pose, '/target_pose/left_leg', 10)
        self.default_quat = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}

        # Timer berjalan di 50 Hz (0.02 detik)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.start_time = time.time()

        # --- PARAMETER LANGKAH ---
        self.stance_width = 0.08   # Jarak antar kaki 8 cm
        self.z_rest = 0.03         # Ketinggian saat menapak di tanah
        self.z_lift = 0.08         # Ketinggian saat melangkah (8 cm, melewati threshold 5 cm)
        self.cycle_time = 4.0      # Total waktu 1 siklus langkah (4 detik)

        self.get_logger().info("🚶 Gait Generator Aktif! Mengirim pola langkah ke kaki...")

    def timer_callback(self):
        # Hitung waktu yang sudah berjalan
        t = time.time() - self.start_time
        
        # Inisialisasi pesan Pose untuk kedua kaki
        pose_r = Pose()
        pose_l = Pose()

        # Posisi X dan Y default (Diam di tempat)
        pose_r.position.y = 0.0
        pose_l.position.y = 0.0
        pose_r.position.x = (self.stance_width ) 
        pose_l.position.x = -(self.stance_width)
        pose_r.position.z = 0.03
        pose_l.position.z = 0.03

        # --- STATE MACHINE (Fase Langkah) ---
        # Membagi waktu menjadi 4 fase berulang
        # phase = t % self.cycle_time

        # if phase < 1.0:
        #     # FASE 1: Kaki Kanan Naik (Menggunakan kurva Sinus agar ayunan mulus)
        #     pose_r.position.z = self.z_lift * math.sin(phase * math.pi)
        #     pose_l.position.z = self.z_rest
            
        # elif phase < 2.0:
        #     # FASE 2: Double Support (Dua kaki napak di tanah)
        #     pose_r.position.z = self.z_rest
        #     pose_l.position.z = self.z_rest
            
        # elif phase < 3.0:
        #     # FASE 3: Kaki Kiri Naik
        #     phase_l = phase - 2.0 # Normalisasi waktu untuk kurva sinus
        #     pose_l.position.z = self.z_lift * math.sin(phase_l * math.pi)
        #     pose_r.position.z = self.z_rest
            
        # else:
        #     # FASE 4: Double Support
        #     pose_r.position.z = self.z_rest
        #     pose_l.position.z = self.z_rest

        pose_r.orientation.w = self.default_quat['w']
        pose_r.orientation.x = self.default_quat['x']
        pose_r.orientation.y = self.default_quat['y']
        pose_r.orientation.z = self.default_quat['z']

        pose_l.orientation.w = self.default_quat['w']
        pose_l.orientation.x = self.default_quat['x']
        pose_l.orientation.y = self.default_quat['y']
        pose_l.orientation.z = self.default_quat['z']

        # Publish target pose ke topik
        self.pub_right.publish(pose_r)
        self.pub_left.publish(pose_l)

def main(args=None):
    rclpy.init(args=args)
    node = WalkingPatternGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
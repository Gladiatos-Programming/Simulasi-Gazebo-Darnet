#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_srvs.srv import Trigger
import subprocess
import time

class FallDetector(Node):
    def __init__(self):
        super().__init__('fall_detector_node')
        
        # 1. Subscriber ke Topic IMU
        self.subscription = self.create_subscription(
            Vector3,
            '/imu',
            self.imu_callback,
            10
        )
        
        # 2. Client untuk mereset memori script IK (Opsional tapi sangat disarankan)
        self.reset_ik_client = self.create_client(Trigger, '/reset_ik_memory')
        
        # 3. Sistem Cooldown (Biar nggak spam command reset pas robot lagi proses jatuh/respawn)
        self.last_reset_time = 0.0
        self.cooldown_duration = 1.0  # Tunggu 5 detik sebelum boleh reset lagi
        
        self.get_logger().info("🛡️ Fall Detector Aktif! Memantau kemiringan di sumbu X (Roll)...")

    def imu_callback(self, msg):
        # Mengecek apakah kemiringan sumbu X lebih dari 80 derajat atau kurang dari -75 derajat
        if msg.x > 75.0 or msg.x < -75.0:
            current_time = time.time()
            
            # Cek apakah sudah melewati masa cooldown
            if (current_time - self.last_reset_time) > self.cooldown_duration:
                self.get_logger().warn(f"🚨 ROBOT JATUH! (Kemiringan X: {msg.x:.2f} derajat). Mereset simulasi...")
                self.reset_simulation()
                self.last_reset_time = time.time()

    def reset_simulation(self):
        # Menggunakan service Set Pose untuk men-teleport robot kembali ke atas (Z=0.2)
        # Posisi tegak lurus (w=1, x=0, y=0, z=0)
        pose_req = 'name: "darnetnew", position: {x: 0.0, y: 0.0, z: 0.2}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}'
        
        cmd_ign = [
            "ign", "service", "-s", "/world/empty/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "3000",
            "--req", pose_req
        ]
        
        try:
            # Teleport robot via Ignition
            subprocess.run(cmd_ign, check=True)
            self.get_logger().info("✅ Robot berhasil di-teleport ke posisi awal!")
        except Exception as e:
            self.get_logger().error(f"❌ Gagal teleport via 'ign': {e}")
            
            # Fallback untuk Gazebo Harmonic/Garden (gz)
            try:
                cmd_gz = [
                    "gz", "service", "-s", "/world/empty/set_pose",
                    "--reqtype", "gz.msgs.Pose",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "3000",
                    "--req", pose_req
                ]
                subprocess.run(cmd_gz, check=True)
                self.get_logger().info("✅ Robot berhasil di-teleport (via gz)!")
            except Exception as e2:
                self.get_logger().error(f"❌ Gagal teleport via 'gz': {e2}")

        # Reset Memori Inverse Kinematics biar robot nggak langsung nekel kakinya pas respawn
        if self.reset_ik_client.wait_for_service(timeout_sec=2.0):
            req = Trigger.Request()
            self.reset_ik_client.call_async(req)
            self.get_logger().info("✅ IK Memory Reset Triggered!")
        else:
            self.get_logger().warn("⚠️ Service /reset_ik_memory tidak ditemukan.")

def main(args=None):
    rclpy.init(args=args)
    node = FallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
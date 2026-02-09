#!/usr/bin/env python3
"""
ROS2 to OpenRB Bridge for Jetson Orin Nano
============================================
Sama seperti ComsROS2OpenRBDARPUT.py, TAPI ditambah:
- Publisher /joint_states agar PinnochioIK bisa jalan TANPA Gazebo
- PinnochioIK butuh /joint_states untuk inisialisasi dan IK computation
- Node ini mem-publish posisi joint yang sudah di-command sebagai feedback

Pipeline di Jetson:
  Motion Script → PinnochioIK → /joint_trajectory → [NODE INI] → Serial → OpenRB
                      ↑                                  |
                /joint_states  ←──────────────────────────┘

Author: Gladiatos 2025
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import serial
import time
import serial.tools.list_ports

OPENRB_VID = 0x2f5d
OPENRB_PID = 0x2202

def find_openrb_port(logger=None):
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if p.vid == OPENRB_VID and p.pid == OPENRB_PID:
            if logger:
                logger.info(f"🔍 OpenRB ditemukan di {p.device}")
            return p.device
    return None

class RosToOpenRBJetson(Node):
    def __init__(self):
        super().__init__('ros_to_openrb_bridge')

        self.last_send_time = 0.0
        self.min_send_interval = 0.02  # 20 ms = 50 Hz

        # --- 1. SETUP SERIAL ---
        self.baud_rate = 115200
        self.ser = None

        # Tunggu sampai OpenRB muncul
        port = None
        while port is None:
            port = find_openrb_port(self.get_logger())
            if port is None:
                self.get_logger().warn("⏳ Menunggu OpenRB...")
                time.sleep(0.5)

        try:
            self.ser = serial.Serial(
                port,
                self.baud_rate,
                timeout=0.05,
                dsrdtr=False,
                rtscts=False
            )
            time.sleep(2.0)  # WAJIB untuk OpenRB
            self.get_logger().info(f"✅ Serial Connected: {port}")
        except Exception as e:
            self.get_logger().error(f"❌ Serial Error: {e}")
            self.ser = None

        # --- 2. MAPPING URUTAN ID 1 S/D 20 ---
        self.ordered_joint_names = [
            'Lengan Kiri',          # ID 1
            'Lengan Kanan',         # ID 2
            'Bahu Tangan Kiri',     # ID 3
            'Bahu Tangan Kanan',    # ID 4
            'Tangan Kiri',          # ID 5
            'Tangan Kanan',         # ID 6
            'Paha Kiri Putar',      # ID 8
            'Paha Atas Kanan',     # ID 7
            'Paha Atas Kiri',       # ID 10
            'Paha Kanan Putar',      # ID 9
            'Paha Bawah Kanan',     # ID 11
            'Paha Bawah Kiri',      # ID 12
            'Lutut Kanan',          # ID 13
            'Lutut Kiri',           # ID 14
            'Kaki Kanan Atas',      # ID 15
            'Kaki Kiri Atas',       # ID 16
            'Kaki Kanan Bawah',     # ID 17
            'Kaki Kiri Bawah',      # ID 18
            'Leher Putar',          # ID 19
            'Kepala Putar'          # ID 20
        ]

        # --- MEMORY POSISI TERAKHIR (PENTING!) ---
        self.last_known_positions = {name: 0.0 for name in self.ordered_joint_names}

        # --- DAFTAR JOINT YANG MAU DI-INVERSE ---
        self.inverted_joints = [
            'Lengan Kiri',
            'Bahu Tangan Kiri',
            'Lengan Kanan',
            'Paha Atas Kanan',
            'Paha Bawah Kanan',
            'Paha Bawah Kiri',
            'Kaki Kiri Bawah',
            'Kaki Kanan Bawah',
            'Kaki Kanan Atas',
            'Kaki Kiri Atas',
            'Paha Kiri Putar',
        ]

        # --- 3. SUBSCRIBER ---
        self.sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.listener_callback,
            10
        )

        # --- 4. STATUS CHECKER + AUTO RECONNECT ---
        self.status_timer = self.create_timer(5.0, self.status_callback)

        # ==============================================================
        # --- 5. JOINT STATE PUBLISHER (PENGGANTI GAZEBO) ---
        # PinnochioIK butuh /joint_states untuk:
        #   a) initialize_targets_once() - inisialisasi awal
        #   b) update_q_from_states()    - update model sebelum IK
        # Tanpa ini, PinnochioIK TIDAK akan memproses target apapun!
        # ==============================================================
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Timer 10Hz untuk publish /joint_states secara berkala
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)

        # Publish sekali langsung di awal agar PinnochioIK bisa inisialisasi
        self.publish_joint_states()

        self.get_logger().info("🤖 JETSON BRIDGE STARTED (with /joint_states publisher)")

    def publish_joint_states(self):
        """Publish posisi joint saat ini sebagai /joint_states feedback ke PinnochioIK"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.last_known_positions.keys())
        msg.position = list(self.last_known_positions.values())
        self.joint_state_pub.publish(msg)

    def listener_callback(self, msg):
        if not msg.points:
            return

        # 1. Update Memory Posisi (nilai asli dari IK, SEBELUM inverse)
        incoming_names = msg.joint_names
        target_point = msg.points[0]
        incoming_positions = target_point.positions

        for name, new_pos in zip(incoming_names, incoming_positions):
            self.last_known_positions[name] = new_pos

        # 2. Susun Data Posisi (20 Angka) untuk serial
        final_values = []
        for name in self.ordered_joint_names:
            val = self.last_known_positions[name]

            # Logika Inverse (hanya untuk serial ke OpenRB)
            if name in self.inverted_joints:
                val = val * -1.0

            final_values.append(val)

        # 3. AMBIL WAKTU (Time From Start)
        sec = target_point.time_from_start.sec
        nano = target_point.time_from_start.nanosec
        time_ms = (sec * 1000.0) + (nano / 1000000.0)

        if time_ms <= 0.0:
            time_ms = 1.0

        final_values.append(time_ms)

        # 4. Kirim ke Arduino
        self.send_serial(final_values)

    def try_reconnect(self):
        """Coba reconnect ke OpenRB jika serial putus"""
        port = find_openrb_port()
        if port is None:
            self.get_logger().warn("🔄 Reconnect: OpenRB belum muncul...")
            return False

        try:
            self.ser = serial.Serial(
                port,
                self.baud_rate,
                timeout=0.05,
                dsrdtr=False,
                rtscts=False
            )
            time.sleep(2.0)
            self.get_logger().info(f"✅ Reconnected: {port}")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Reconnect gagal: {e}")
            self.ser = None
            return False

    def status_callback(self):
        """Kirim command status ke OpenRB dan baca response"""
        # Auto-reconnect jika serial putus
        if not (self.ser and self.ser.is_open):
            self.try_reconnect()
            return

        try:
            self.ser.write(b"status\n")

            start_time = time.time()
            while time.time() - start_time < 0.5:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.get_logger().info(f"[SERVO] {line}")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Serial lost (status): {e}")
            try:
                self.ser.close()
            except:
                pass
            self.ser = None

    def send_serial(self, values):
        now = time.time()
        if now - self.last_send_time < self.min_send_interval:
            return

        self.last_send_time = now

        if self.ser and self.ser.is_open:
            msg_str = ",".join([f"{v:.4f}" for v in values]) + "\n"
            try:
                self.ser.write(msg_str.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f"❌ Serial lost: {e}")
                try:
                    self.ser.close()
                except:
                    pass
                self.ser = None


def main(args=None):
    rclpy.init(args=args)
    node = RosToOpenRBJetson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

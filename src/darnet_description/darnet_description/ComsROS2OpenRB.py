#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# Ubah import message
from trajectory_msgs.msg import JointTrajectory 
import serial
import time

class RosToOpenRB(Node):
    def __init__(self):
        super().__init__('ros_to_openrb_bridge')
        
        # --- 1. SETUP SERIAL ---
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.05)
            self.get_logger().info(f"✅ Serial Connected: {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"❌ Serial Error: {e}")
            self.ser = None

        # --- 2. MAPPING URUTAN ID 1 S/D 20 ---
        # Ini adalah "Kunci Jawaban" urutan servo kamu
        self.ordered_joint_names = [
            'Lengan Kiri',          # ID 1
            'Lengan Kanan',         # ID 2
            'Bahu Tangan Kiri',     # ID 3
            'Bahu Tangan Kanan',    # ID 4
            'Tangan Kiri',          # ID 5
            'Tangan Kanan',         # ID 6
            'Paha Atas Kanan',     # ID 7
            'Paha Kiri Putar',      # ID 8
            'Paha Kanan Putar',      # ID 9
            'Paha Atas Kiri',       # ID 10
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
        # Kita simpan posisi terakhir setiap servo di sini.
        # Awalnya 0.0, tapi nanti akan terisi seiring waktu.
        # Dictionary format: {'Nama Joint': Nilai Terakhir}
        self.last_known_positions = {name: 0.0 for name in self.ordered_joint_names}

        # --- DAFTAR JOINT YANG MAU DI-INVERSE ---
        self.inverted_joints = [
            'Lengan Kiri',
            'Bahu Tangan Kiri',
            'Lengan Kanan',
            'Paha Atas Kanan',
            'Paha Bawah Kanan',
            'Paha Bawah Kiri',
            'Lutut Kiri',
            'Lutut Kanan',
            'Kaki Kiri Bawah',
            'Kaki Kanan Bawah',
        ]

        # --- 3. SUBSCRIBER ---
        # Subscribe ke JointTrajectory
        self.sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        if not msg.points:
            return 

        # 1. Update Memory Posisi
        incoming_names = msg.joint_names
        # Ambil point pertama (Immediate Target)
        target_point = msg.points[0]
        incoming_positions = target_point.positions

        for name, new_pos in zip(incoming_names, incoming_positions):
            self.last_known_positions[name] = new_pos

        # 2. Susun Data Posisi (20 Angka)
        final_values = []
        for name in self.ordered_joint_names:
            val = self.last_known_positions[name]
            
            # Logika Inverse
            if name in self.inverted_joints:
                val = val * -1.0
            
            final_values.append(val)

        # 3. AMBIL WAKTU (Time From Start)
        # Convert detik dan nanodetik menjadi Milidetik (ms)
        sec = target_point.time_from_start.sec
        nano = target_point.time_from_start.nanosec
        
        # Rumus: (Detik * 1000) + (Nano / 1.000.000)
        time_ms = (sec * 1000.0) + (nano / 1000000.0)
        
        # Safety: Kalau waktunya 0 (instan), kita set minimal 1ms biar gak error pembagian
        if time_ms <= 0.0:
            time_ms = 1.0 # Default speed cepat

        # Masukkan waktu ke list data terakhir (Item ke-21)
        final_values.append(time_ms)

        # 4. Kirim ke Arduino
        self.send_serial(final_values)

    def send_serial(self, values):
        if self.ser and self.ser.is_open:
            # Format: "0.00,1.57,-3.14,..."
            msg_str = ",".join([f"{v:.4f}" for v in values]) + "\n"
            
            try:
                self.ser.write(msg_str.encode('utf-8'))
                # Debug print (Opsional)
                # self.get_logger().info(f"Sending: {msg_str.strip()}")
            except Exception as e:
                self.get_logger().warn(f"Serial Write Fail: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RosToOpenRB()
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

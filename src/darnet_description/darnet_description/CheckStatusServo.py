#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time

class StatusChecker(Node):
    def __init__(self):
        super().__init__('status_checker_node')
        
        # --- SETUP SERIAL ---
        self.port = '/dev/ttyACM0' # Sesuaikan
        self.baud = 115200
        
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
            self.ser.flush()
            self.get_logger().info(f"✅ Terhubung ke {self.port}")
        except Exception as e:
            self.get_logger().error(f"❌ Gagal Serial: {e}")
            self.ser = None

        # Timer: Kirim command setiap 2 detik
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        if self.ser and self.ser.is_open:
            # 1. Kirim Command "status"
            cmd = "status\n"
            self.ser.write(cmd.encode('utf-8'))
            self.get_logger().info("Mengirim: [ status ]")
            
            # 2. Baca Balasan dari Arduino
            # Kita kasih delay dikit biar Arduino sempat memproses
            time.sleep(0.5) 
            
            while self.ser.in_waiting > 0:
                try:
                    # Baca baris per baris
                    line = self.ser.readline().decode('utf-8').strip()
                    
                    if line:
                        self.get_logger().info(f"Diterima: {line}")
                        
                except UnicodeDecodeError:
                    pass
        else:
            self.get_logger().warn("Serial putus/belum connect")

def main(args=None):
    rclpy.init(args=args)
    node = StatusChecker()
    
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
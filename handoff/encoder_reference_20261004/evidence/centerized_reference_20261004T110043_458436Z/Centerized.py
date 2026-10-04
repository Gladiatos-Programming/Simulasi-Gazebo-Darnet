#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time
import threading # PENTING: Untuk memisahkan eksekusi ROS dan Logika Gerak

class StandUpController(Node):
    def __init__(self):
        super().__init__('standup_controller')
        
        # Gunakan ReentrantCallbackGroup agar aman jika nanti butuh multi-threaded callbacks
        self.cb_group = ReentrantCallbackGroup()

        # Publisher untuk joint trajectory
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
            callback_group=self.cb_group
        )
        
        # Subscribe ke joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.cb_group
        )
        
        self.current_joint_positions = {}
        self.data_received = False # Flag untuk memastikan data sudah masuk
        self.get_logger().info('Stand Up Controller Started!')

    def joint_state_callback(self, msg):
        """Update posisi joint saat ini (Berjalan di background thread)"""
        self.data_received = True
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
    
    def wait_for_connection(self):
        """Mencegah race condition: Tunggu sampai ada subscriber (Controller aktif)"""
        self.get_logger().info('Waiting for trajectory controller connection...')
        while self.publisher.get_subscription_count() == 0:
            time.sleep(0.5)
        self.get_logger().info('✅ Controller connected!')

        """Tunggu sampai data joint state pertama masuk"""
        self.get_logger().info('Waiting for joint states data...')
        while not self.data_received:
            time.sleep(0.1)
        self.get_logger().info('✅ Joint states received!')

    def move_joints_smooth(self, joint_names, positions_list, duration_per_point=2.0):
        msg = JointTrajectory()
        msg.joint_names = joint_names
        
        # Validasi sederhana untuk mencegah memory error jika list kosong
        if not positions_list:
            return

        for i, positions in enumerate(positions_list):
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions] # Pastikan float untuk efisiensi memori
            
            # Hitung waktu akumulatif
            time_from_start = (i + 1) * duration_per_point
            sec = int(time_from_start)
            nanosec = int((time_from_start - sec) * 1e9)
            
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            msg.points.append(point)
        
        self.publisher.publish(msg)
        self.get_logger().info(f'🚀 Publishing trajectory: {len(positions_list)} waypoints over {duration_per_point}s each')
    
    def standup_sequence(self):
        self.get_logger().info('='*50)
        self.get_logger().info('STAND UP SEQUENCE STARTED')
        self.get_logger().info('='*50)
        
        # Pastikan koneksi aman sebelum mulai (Anti Race Condition)
        self.wait_for_connection()

        all_joints = [
            'Paha Kiri Putar', 'Paha Kanan Putar',
            'Paha Atas Kiri', 'Paha Atas Kanan',
            'Paha Bawah Kiri', 'Paha Bawah Kanan',
            'Lutut Kiri', 'Lutut Kanan',
            'Kaki Kiri Atas', 'Kaki Kanan Atas',
            'Kaki Kiri Bawah', 'Kaki Kanan Bawah',
            'Lengan Kiri', 'Lengan Kanan',
            'Bahu Tangan Kiri', 'Bahu Tangan Kanan',
            'Tangan Kiri', 'Tangan Kanan',
            'Leher Putar', 'Kepala Putar'
        ]
        
        # ============================================================
        # PHASE 1: PREPARE
        # ============================================================
        self.get_logger().info('\n[PHASE 1] Preparing position to centerized...')
        
        target_positions = [
             0.0, 0.0,          # Pinggul putar
             0.0, 0.0,          # Paha atas
             0.0, 0.0,          # Paha bawah
             0.0, 0.0,          # Lutut
             0.0, 0.0,          # Kaki atas
             0.0, 0.0,          # Kaki bawah
             0.0, 0.0,          # Lengan
             0.0, 0.0,          # Bahu
             0.0, 0.0,          # Tangan
             0.0, 0.0           # Kepala
        ]

        self.move_joints_smooth(
            all_joints,
            [target_positions], # Waypoint 1
            1.0                 # Durasi 1 detik
        )
        
        # Sleep aman dilakukan disini karena spin() jalan di thread lain
        time.sleep(3.5)
        
        self.get_logger().info('Sequence Finished.')

def main(args=None):
    rclpy.init(args=args)
    controller = StandUpController()
    
    # --- SOLUSI RACE CONDITION ---
    # Kita jalankan rclpy.spin di thread terpisah.
    # Ini membuat callback (penerimaan data sensor) tetap jalan di background
    # sementara kode utama (standup_sequence) jalan di foreground.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        # Jalankan logika utama
        controller.standup_sequence()
        
        # Keep alive jika perlu memantau joint states setelah gerakan selesai
        # Atau langsung exit jika script cuma one-shot
        time.sleep(1.0) 
        
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
    finally:
        controller.get_logger().info('Shutting down...')
        controller.destroy_node()
        rclpy.shutdown()
        # Thread daemon akan otomatis mati saat main program mati

if __name__ == '__main__':
    main()
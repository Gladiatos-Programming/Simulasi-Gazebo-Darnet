#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState, Imu
from builtin_interfaces.msg import Duration
import time
import math

# NOTE: Untuk yang dapet kode ini ini adalah kode buat darnet berdiri dari jatoh ke depan yang ada di simulasi -Gladiatos 2025
class StandUpController(Node):
    def __init__(self):
        super().__init__('standup_controller')
        
        # Publisher untuk joint trajectory
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Subscribe ke joint states untuk monitoring
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_positions = {}
        self.get_logger().info('Stand Up Controller Started!')
        
        time.sleep(1)  # Wait for connections
        
    def joint_state_callback(self, msg):
        """Update posisi joint saat ini"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
    
    def move_joints_smooth(self, joint_names, positions_list, duration_per_point=2.0):
        """
        Gerakkan joint melalui multiple waypoints
        
        Args:
            joint_names: list nama joint
            positions_list: list of lists, misal [[pos1, pos2], [pos3, pos4], ...]
            duration_per_point: durasi per waypoint (detik)
        """
        msg = JointTrajectory()
        msg.joint_names = joint_names
        
        for i, positions in enumerate(positions_list):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=int((i+1) * duration_per_point))
            msg.points.append(point)
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Moving {len(joint_names)} joints through {len(positions_list)} waypoints')
    
    def standup_sequence(self):
        """
        Sequence lengkap untuk berdiri dari tengkurap
        
        Tahapan:
        1. Tekuk lutut (prepare)
        2. Angkat pinggul ke atas
        3. Dorong dengan tangan
        4. Pindahkan center of mass
        5. Luruskan kaki
        6. Posisi berdiri stabil
        """
        
        self.get_logger().info('='*50)
        self.get_logger().info('STAND UP SEQUENCE STARTED')
        self.get_logger().info('='*50)
        
        # Definisi joint groups
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
        # PHASE 1: PREPARE - Tekuk lutut dan posisikan tangan
        # ============================================================
        self.get_logger().info('\n[PHASE 1] Preparing position...')
        
        self.move_joints_smooth(
            all_joints,
            [
                # Tekuk lutut, tangan siap push
                [0.0, 0.0,           # Pinggul putar (netral)
                 0.0, 0.0,         # Paha atas (sedikit tekuk)
                 1.647, -1.647,           # Paha bawah
                 0.0, 0.0,         # Lutut (tekuk 80°)
                 0.0, 0.0,           # Kaki atas (tekuk)
                 0.0, 0.0,           # Kaki bawah
                 -1.4, 1.4,          # Lengan (posisi push)
                 0.0, 0.0,          # Bahu (push position)
                 0.0, 0.0,          # Tangan
                 0.0, 0.0]           # Kepala
            ],
            1.0 # 3 detik
        )
        time.sleep(2.0)
        
        # ============================================================
        # PHASE 2: PUSH UP - Angkat badan dengan tangan
        # ============================================================
        self.get_logger().info('\n[PHASE 2] Pushing up with arms...')
        
        self.move_joints_smooth(
            all_joints,
            [
                # Push dengan tangan, angkat pinggul
                [0.0, 0.0,           # Pinggul putar (netral)
                 0.0, 0.0,         # Paha atas (sedikit tekuk)
                 1.647, -1.647,           # Paha bawah
                 0.0, 0.0,         # Lutut (tekuk 80°)
                 0.0, 0.0,           # Kaki atas (tekuk)
                 0.0, 0.0,           # Kaki bawah
                 -1.4, 1.4,          # Lengan (posisi push)
                 0.0, 0.0,          # Bahu (push position)
                 3.0, -3.0,          # Tangan
                 0.0, 0.0]           # Kepala
            ],
            1.0
        )
        time.sleep(2.0)
        
        # ============================================================
        # PHASE 3: TRANSITION - Pindah ke posisi kneel (berlutut)
        # ============================================================
        self.get_logger().info('\n[PHASE 3] Transitioning to kneeling...')
        
        self.move_joints_smooth(
            all_joints,
            [
                # Tarik kaki ke depan, posisi berlutut
                # Push dengan tangan, angkat pinggul
                [0.0, 0.0,           # Pinggul putar (netral)
                 0.0, 0.0,         # Paha atas (sedikit tekuk)
                 1.647, -1.647,           # Paha bawah
                 0.0, 0.0,         # Lutut (tekuk 80°)
                 0.0, 0.0,           # Kaki atas (tekuk)
                 0.0, 0.0,           # Kaki bawah
                 0.02, -0.02,          # Lengan (posisi push)
                 0.0, 0.0,          # Bahu (push position)
                 0.7, -0.7,           # Tangan
                 0.0, 0.0]           # Kepala
            ],
            1.0
        )
        time.sleep(2.0)

        

        # ============================================================
        # PHASE 4: HALF STAND - Satu kaki maju
        # ============================================================
        self.get_logger().info('\n[PHASE 4] aaaaaadadadadada...')
        
        self.move_joints_smooth(
            all_joints,
            [
                # Kaki kanan maju, bersiap berdiri
                # Push dengan tangan, angkat pinggul
                [0.0, 0.0,           # Pinggul putar (netral)
                 0.0, 0.0,         # Paha atas (sedikit tekuk)
                 -1.8, 1.8,           # Paha bawah
                 2.3, -2.3,         # Lutut (tekuk 80°)
                 -1.72, 1.72,           # Kaki atas (tekuk)
                 0.0, 0.0,           # Kaki bawah
                 0.7, -0.7,          # Lengan (posisi push)
                 0.0, 0.0,          # Bahu (push position)
                 1.2, -1.2,          # Tangan
                 0.0, 0.0]           # Kepala
            ],
            1.0
        )
        time.sleep(2.0)


        self.get_logger().info('\n[PHASE 4] bbbbbbbbbbb...')
        
        self.move_joints_smooth(
            all_joints,
            [
                [0.0, 0.0,           # Pinggul putar (netral)
                 0.0, 0.0,         # Paha atas (sedikit tekuk)
                 -2.1, 2.1,           # Paha bawah
                 2.35, -2.35,         # Lutut (tekuk 80°)
                 -1.2, 1.2,           # Kaki atas (tekuk)
                 0.0, 0.0,           # Kaki bawah
                 0.7, -0.7,          # Lengan (posisi push)
                 0.0, 0.0,          # Bahu (push position)
                 0.8, -0.8,          # Tangan
                 0.0, 0.0]           # Kepala
            ],
            1.0
        )
        time.sleep(2.0)


        
        # # ============================================================
        # # PHASE 5: PUSH TO STAND - Dorong dengan kaki depan
        # # ============================================================
        self.get_logger().info('\n[PHASE 5] Standing up...')
        
        self.move_joints_smooth(
            all_joints,
            [
                [0.0, 0.0,           # Pinggul putar (netral)
                 0.0, 0.0,         # Paha atas (sedikit tekuk)
                 -2.1, 2.1,           # Paha bawah
                 2.35, -2.35,         # Lutut (tekuk 80°)
                 -0.9, 0.9,           # Kaki atas (tekuk)
                 0.0, 0.0,           # Kaki bawah
                 0.7, -0.7,          # Lengan (posisi push)
                 0.0, 0.0,          # Bahu (push position)
                 0.8, -0.8,          # Tangan
                 0.0, 0.0]           # Kepala
            ],
            1.0
        )
        time.sleep(2.0)
        
        # ============================================================
        # PHASE 6: STABILIZE - Stabilkan posisi berdiri
        # ============================================================
        self.get_logger().info('\n[PHASE 6] Stabilizing...')
        
        self.move_joints_smooth(
            all_joints,
            [
                # Kaki kanan maju, bersiap berdiri
                [0.0, 0.0,           # Pinggul putar (netral)
                 0.0, 0.0,         # Paha atas (sedikit tekuk)
                 -0.4, 0.4,           # Paha bawah
                 0.5, -0.5,         # Lutut (tekuk 80°)
                 -0.17, 0.17,           # Kaki atas (tekuk)
                 0.0, 0.0,           # Kaki bawah
                 -0.1, 0.1,          # Lengan (posisi push)
                 0.0, 0.0,          # Bahu (push position)
                 1.0, -1.0,          # Tangan
                 0.0, 0.0]           # Kepala
            ],
            1.0
        )
        time.sleep(3)
        
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('STAND UP COMPLETED!')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    controller = StandUpController()
    
    try:
        # Tunggu sebentar untuk joint states ready
        controller.get_logger().info('Waiting for joint states...')
        time.sleep(2)
        
        # Execute stand up sequence
        controller.standup_sequence()
        
        controller.get_logger().info('\nStand up sequence finished!')
        
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
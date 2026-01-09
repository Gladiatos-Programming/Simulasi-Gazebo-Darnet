#!/usr/bin/env python3
"""
Robot Initialization Controller
================================
Script ini menggerakkan robot ke posisi awal dengan:
- Kaki menggunakan Inverse Kinematics (target pose)
- Tangan menggunakan joint angles langsung (manual)

Author: Gladiatos 2025
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
import time

class RobotInitController(Node):
    def __init__(self):
        super().__init__('robot_init_controller')
        
        # ====================================================================
        # PUBLISHERS & SUBSCRIBERS
        # ====================================================================
        
        # Publisher untuk manual joint commands (tangan, kepala)
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Publisher untuk IK targets (kaki)
        self.ik_right_leg_pub = self.create_publisher(
            Pose, 
            '/target_pose/right_leg', 
            10
        )
        self.ik_left_leg_pub = self.create_publisher(
            Pose, 
            '/target_pose/left_leg', 
            10
        )
        
        # Subscriber untuk monitoring
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_positions = {}
        
        # ====================================================================
        # KONFIGURASI TARGET POSISI
        # ====================================================================
        
        # Target IK untuk KAKI KANAN (dalam meter, world frame)
        self.target_right_leg = {
            'position': {'x': 0.2, 'y': -0.06, 'z': 0.01},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        }
        
        # Target IK untuk KAKI KIRI (dalam meter, world frame)
        self.target_left_leg = {
            'position': {'x': 0.04, 'y': -0.06, 'z': 0.01},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        }
        
        # Target MANUAL untuk TANGAN & KEPALA (dalam radian)
        self.target_arms_head = {
            'Lengan Kiri': 0.3,
            'Lengan Kanan': -0.3,
            'Bahu Tangan Kiri': 0.0,
            'Bahu Tangan Kanan': 0.0,
            'Tangan Kiri': 1.2,
            'Tangan Kanan': -1.2,
            'Leher Putar': 0.0,
            'Kepala Putar': 0.0
        }
        
        self.get_logger().info('✅ Robot Initialization Controller Ready!')
        self.get_logger().info('   - Right Leg IK Target: ' + 
                              f"({self.target_right_leg['position']['x']:.2f}, " +
                              f"{self.target_right_leg['position']['y']:.2f}, " +
                              f"{self.target_right_leg['position']['z']:.2f})")
        self.get_logger().info('   - Left Leg IK Target: ' + 
                              f"({self.target_left_leg['position']['x']:.2f}, " +
                              f"{self.target_left_leg['position']['y']:.2f}, " +
                              f"{self.target_left_leg['position']['z']:.2f})")
        
        # Wait untuk koneksi
        time.sleep(1.0)

    def joint_state_callback(self, msg):
        """Update posisi joint saat ini untuk monitoring"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = msg.position[i]

    def send_leg_ik_targets(self):
        """
        Kirim target IK ke Pinocchio untuk kedua kaki
        Pinocchio IK node akan menghitung joint angles yang diperlukan
        """
        self.get_logger().info('\n🦵 STEP 1: Sending IK Targets for Legs...')
        
        # Kaki Kanan
        msg_right = Pose()
        msg_right.position.x = self.target_right_leg['position']['x']
        msg_right.position.y = self.target_right_leg['position']['y']
        msg_right.position.z = self.target_right_leg['position']['z']
        msg_right.orientation.x = self.target_right_leg['orientation']['x']
        msg_right.orientation.y = self.target_right_leg['orientation']['y']
        msg_right.orientation.z = self.target_right_leg['orientation']['z']
        msg_right.orientation.w = self.target_right_leg['orientation']['w']
        
        self.ik_right_leg_pub.publish(msg_right)
        self.get_logger().info(f'   ✅ Right leg target published')
        
        # Kaki Kiri
        msg_left = Pose()
        msg_left.position.x = self.target_left_leg['position']['x']
        msg_left.position.y = self.target_left_leg['position']['y']
        msg_left.position.z = self.target_left_leg['position']['z']
        msg_left.orientation.x = self.target_left_leg['orientation']['x']
        msg_left.orientation.y = self.target_left_leg['orientation']['y']
        msg_left.orientation.z = self.target_left_leg['orientation']['z']
        msg_left.orientation.w = self.target_left_leg['orientation']['w']
        
        self.ik_left_leg_pub.publish(msg_left)
        self.get_logger().info(f'   ✅ Left leg target published')

    def send_arms_head_commands(self, duration=2.0):
        """
        Kirim command manual untuk tangan dan kepala
        
        Args:
            duration: Waktu eksekusi gerakan (detik)
        """
        self.get_logger().info('\n💪 STEP 2: Sending Manual Commands for Arms & Head...')
        
        # Buat trajectory message
        msg = JointTrajectory()
        msg.joint_names = list(self.target_arms_head.keys())
        
        # Buat trajectory point
        point = JointTrajectoryPoint()
        point.positions = list(self.target_arms_head.values())
        point.time_from_start = Duration(sec=int(duration))
        
        msg.points.append(point)
        
        # Publish
        self.joint_pub.publish(msg)
        
        self.get_logger().info(f'   ✅ Arms & Head commands published')
        self.get_logger().info(f'      Joints: {msg.joint_names}')
        self.get_logger().info(f'      Positions (rad): {[f"{p:.2f}" for p in point.positions]}')
        self.get_logger().info(f'      Duration: {duration}s')

    def execute_initialization(self):
        """
        Main execution function
        Sequence:
        1. Kirim target IK untuk kaki
        2. Tunggu IK compute & execute
        3. Kirim command manual untuk tangan & kepala
        4. Tunggu selesai
        """
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('🤖 ROBOT INITIALIZATION SEQUENCE')
        self.get_logger().info('='*70)
        
        # ================================================================
        # STEP 1: IK untuk Kaki
        # ================================================================
        self.send_leg_ik_targets()
        
        # Tunggu IK selesai
        # Breakdown:
        # - IK compute time: ~0.1-0.5s
        # - IK execution: 1.0s (dari IK node Duration)
        # - Safety buffer: 1.5s
        # Total: 3.0s
        ik_wait_time = 3.0
        self.get_logger().info(f'\n⏳ Waiting {ik_wait_time}s for IK to complete...')
        
        for i in range(int(ik_wait_time * 2)):  # Update setiap 0.5s
            time.sleep(0.5)
            progress = (i + 1) * 0.5
            bar_length = 20
            filled = int(bar_length * progress / ik_wait_time)
            bar = '█' * filled + '░' * (bar_length - filled)
            self.get_logger().info(f'   [{bar}] {progress:.1f}s / {ik_wait_time}s')
        
        self.get_logger().info('   ✅ IK execution completed!\n')
        
        # ================================================================
        # STEP 2: Manual Commands untuk Tangan & Kepala
        # ================================================================
        arm_duration = 2.0
        self.send_arms_head_commands(duration=arm_duration)
        
        # Tunggu arm movement selesai
        arm_wait_time = arm_duration + 1.0  # Duration + buffer
        self.get_logger().info(f'\n⏳ Waiting {arm_wait_time}s for arm movement to complete...')
        time.sleep(arm_wait_time)
        self.get_logger().info('   ✅ Arm movement completed!\n')
        
        # ================================================================
        # SELESAI
        # ================================================================
        self.get_logger().info('='*70)
        self.get_logger().info('✅ INITIALIZATION SEQUENCE COMPLETED!')
        self.get_logger().info('='*70)
        self.get_logger().info('\n📊 Final Status:')
        self.get_logger().info('   - Legs positioned via IK ✅')
        self.get_logger().info('   - Arms & Head positioned ✅')
        self.get_logger().info('   - Robot ready for next task! 🚀\n')


def main(args=None):
    rclpy.init(args=args)
    controller = RobotInitController()
    
    try:
        # Tunggu sebentar untuk semua koneksi ready
        controller.get_logger().info('⏳ Initializing ROS connections...')
        time.sleep(1.5)
        
        # Execute main sequence
        controller.execute_initialization()
        
        # Keep node alive untuk monitoring (optional)
        controller.get_logger().info('Node will shutdown in 3 seconds...')
        time.sleep(3.0)
        
    except KeyboardInterrupt:
        controller.get_logger().info('\n⚠️  Interrupted by user')
    except Exception as e:
        controller.get_logger().error(f'\n❌ Error occurred: {e}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
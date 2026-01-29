#!/usr/bin/env python3
import os
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
import numpy as np
import pinocchio as pin
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import xacro
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Int32, Int64
from std_srvs.srv import Trigger

class DiagnosticIKCalculator(Node):
    def __init__(self):
        super().__init__('diagnostic_ik_calculator')
        
        # --- 1. CONFIG GLOBAL ---
        self.STANCE_WIDTH = 0.12 
        
        # --- 2. LOAD MODEL ---
        try:
            share_dir = get_package_share_directory('darnet_description')
            xacro_file = os.path.join(share_dir, 'urdf', 'darnet.xacro')
            doc = xacro.process_file(xacro_file)
            urdf_xml = doc.toxml()
            self.model = pin.buildModelFromXML(urdf_xml)
            self.data = self.model.createData()
            self.get_logger().info("✅ Model Loaded.")
        except Exception as e:
            self.get_logger().error(f"❌ Error loading URDF: {e}")
            raise e

        # --- 3. STATE MEMORY (SOLUSI MASALAH KAMU) ---
        # Kita simpan "Target Terakhir" untuk setiap joint di seluruh robot
        self.global_joint_targets = {} 
        
        # Inisialisasi awal target dengan posisi Neutral (Nol)
        for i in range(self.model.nq):
            name = self.model.names[i]
            # Abaikan 'universe'
            if name != "universe":
                self.global_joint_targets[name] = 0.0

        # --- 4. CONFIG GROUPS ---
        self.target_config = {
            'RightArm': {
                'joints': ['Bahu Tangan Kanan', 'Lengan Kanan', 'Tangan Kanan'], 
                'ee_link': 'End_Effector_Tangan_Kanan_1',
                'topic': '/target_pose/right_arm',
                'solve_rotation': False 
            },
            'RightLeg': {
                'joints': ['Paha Kanan Putar','Paha Atas Kanan', 'Paha Bawah Kanan', 'Lutut Kanan', 'Kaki Kanan Atas', 'Kaki Kanan Bawah'], 
                'ee_link': 'End_Effector_Kaki_Kanan_1', 
                'topic': '/target_pose/right_leg',
                'solve_rotation': True
            },
            'LeftLeg': {
                'joints': ['Paha Kiri Putar','Paha Atas Kiri', 'Paha Bawah Kiri', 'Lutut Kiri', 'Kaki Kiri Atas', 'Kaki Kiri Bawah'], 
                'ee_link': 'End_Effector_Kaki_Kiri_1', 
                'topic': '/target_pose/left_leg',
                'solve_rotation': True
            },
            'LeftArm': {
                'joints': ['Bahu Tangan Kiri', 'Lengan Kiri', 'Tangan Kiri'], 
                'ee_link': 'End_Effector_Tangan_Kiri_1',
                'topic': '/target_pose/left_arm',
                'solve_rotation': False
            }
        }

        # --- 5. PRE-PROCESS GROUPS ---
        self.groups_data = {} 
        self.all_monitored_joints = [] 

        for group_name, config in self.target_config.items():
            joint_names = config['joints']
            joint_ids = []
            
            for name in joint_names:
                if self.model.existJointName(name):
                    joint_ids.append(self.model.getJointId(name))
                    if name not in self.all_monitored_joints:
                        self.all_monitored_joints.append(name)
                else:
                    self.get_logger().error(f"❌ Joint '{name}' NOT FOUND!")
            
            ee_name = config['ee_link']
            if self.model.existFrame(ee_name):
                ee_frame_id = self.model.getFrameId(ee_name)
            else:
                ee_frame_id = -1

            self.groups_data[group_name] = {
                'joint_names': joint_names,
                'joint_ids': joint_ids,
                'ee_frame_id': ee_frame_id,
                'solve_rotation': config['solve_rotation']
            }
            self.create_subscription(
                Pose, config['topic'], 
                lambda msg, g=group_name: self.callback_generic_target(msg, g), 10
            )
            self.get_logger().info(f"✅ {group_name} Ready.")

        # --- 6. ROS COMMS ---
        self.joint_state_received = False
        self.current_joint_states = {}
        
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.joint_cmd_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.q_current = pin.neutral(self.model)
        
        # Timer untuk inisialisasi awal target dari joint state nyata
        self.init_timer = self.create_timer(1.0, self.initialize_targets_once)
        self.initialized = False


        self.current_duration_ns = 50000000 
        
        # Subscriber untuk mengubah speed dari node lain
        self.speed_sub = self.create_subscription(
            Int64, 
            '/servo_speed_ns', 
            self.callback_change_speed, 
            10
        )
        self.callback_change_speed(Int64(data=self.current_duration_ns))  # Set default speed

        self.reset_service = self.create_service(
            Trigger, 
            '/reset_ik_memory', 
            self.callback_reset_memory
        )
        self.get_logger().info("🔘 Reset Service Ready: /reset_ik_memory")


    def callback_change_speed(self, msg):
        """Menerima request durasi dalam nanoseconds"""
        self.current_duration_ns = msg.data
        self.get_logger().info(f"🐢 Speed Updated to: {self.current_duration_ns} ns")

    def callback_reset_memory(self, request, response):
        """Fungsi ini dipanggil saat service /reset_ik_memory dijalankan"""
        self.get_logger().warn("♻️ RESETTING MEMORY...")
        
        # 1. Update posisi robot terbaru dari sensor
        self.update_q_from_states()
        
        # 2. Timpa semua target di memori dengan posisi saat ini
        if self.current_joint_states:
            for name, pos in self.current_joint_states.items():
                self.global_joint_targets[name] = pos
            
            response.success = True
            response.message = "Memory Reset & Synced to Current Pose!"
            self.get_logger().info("✅ Memory Reset Done.")
        else:
            response.success = False
            response.message = "Failed: No Joint States received yet!"
            self.get_logger().error("❌ Cannot Reset: No Joint States!")
            
        return response



    def initialize_targets_once(self):
        """Ambil posisi robot saat ini sebagai target awal agar tidak kaget saat start"""
        if self.joint_state_received and not self.initialized:
            for name, pos in self.current_joint_states.items():
                self.global_joint_targets[name] = pos
            self.initialized = True
            self.init_timer.cancel()
            self.get_logger().info("✅ Global Targets Initialized from Current State.")

    def joint_state_callback(self, msg):
        self.joint_state_received = True
        for i, name in enumerate(msg.name):
            # Update posisi saat ini untuk keperluan IK Calculation
            if name in self.all_monitored_joints:
                self.current_joint_states[name] = msg.position[i]

    def update_q_from_states(self):
        if not self.current_joint_states: return
        for name, pos in self.current_joint_states.items():
            if self.model.existJointName(name):
                jid = self.model.getJointId(name)
                idx_q = self.model.joints[jid].idx_q
                self.q_current[idx_q] = pos

    def callback_generic_target(self, msg, group_name):
        if not self.initialized:
            self.get_logger().warn("⚠️ Tunggu inisialisasi joint state...")
            return

        target_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        target_quat = pin.Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        target_SE3 = pin.SE3(target_quat.matrix(), target_pos)

        # Update model internal dengan posisi real robot saat ini sebelum hitung IK
        self.update_q_from_states()
        
        # Hitung IK hanya untuk group yang dipanggil
        self.solve_and_update_global(group_name, target_SE3)

    def solve_and_update_global(self, group_name, target_SE3):
        group_info = self.groups_data[group_name]
        
        # 1. Hitung Solusi IK
        q_solusi, success, err = self.compute_ik(
            target_SE3, 
            group_info['joint_ids'], 
            group_info['ee_frame_id'],
            group_info['solve_rotation']
        )
        
        status = "✅" if success else "⚠️ (LIMIT)"
        self.get_logger().info(f"🎯 {group_name} Updated: {status} Err: {err:.4f}")

        # 2. UPDATE MEMORY GLOBAL (Hanya joint milik grup ini yang diupdate)
        for i, j_name in enumerate(group_info['joint_names']):
            j_id = self.model.getJointId(j_name)
            idx_q = self.model.joints[j_id].idx_q
            # Simpan nilai target baru ke memory global
            self.global_joint_targets[j_name] = float(q_solusi[idx_q])

        # 3. PUBLISH SEMUA JOINT (Lama + Baru)
        self.publish_all_joints()

    def publish_all_joints(self):
        """Mengirim perintah untuk SEMUA joint yang terdaftar, bukan cuma yang baru diupdate"""
        msg = JointTrajectory()
        
        # Kumpulkan semua joint yang pernah kita sentuh/monitor
        all_active_joints = self.all_monitored_joints
        msg.joint_names = all_active_joints
        
        point = JointTrajectoryPoint()
        target_values = []
        
        for name in all_active_joints:
            # Ambil nilai dari Memory Global. 
            # Jika joint ini tidak sedang diupdate, dia akan pakai nilai terakhir (diam/lanjut gerak).
            val = self.global_joint_targets.get(name, 0.0)
            target_values.append(val)
            
        point.positions = target_values
        point.time_from_start = Duration(sec=0, nanosec=self.current_duration_ns) 
        msg.points.append(point)
        
        self.joint_cmd_pub.publish(msg)

    def compute_ik(self, target_SE3, joint_ids, ee_frame_id, solve_rotation):
        # ... (SAMA SEPERTI SEBELUMNYA, TIDAK ADA PERUBAHAN DI ALGORITMA IK) ...
        q = self.q_current.copy()
        eps = 1e-3 
        max_iter = 500
        dt = 0.1
        damp = 1e-3

        success = False
        final_err = 0.0

        for i in range(max_iter):
            pin.framesForwardKinematics(self.model, self.data, q)
            current_SE3 = self.data.oMf[ee_frame_id]
            
            if solve_rotation:
                error_se3 = current_SE3.actInv(target_SE3)
                err_vec = pin.log(error_se3).vector 
            else:
                err_vec = target_SE3.translation - current_SE3.translation

            final_err = np.linalg.norm(err_vec)
            if final_err < eps:
                return q, True, final_err

            J = pin.computeFrameJacobian(self.model, self.data, q, ee_frame_id, pin.ReferenceFrame.LOCAL)
            
            if not solve_rotation:
                J = pin.computeFrameJacobian(self.model, self.data, q, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                J = J[:3, :] 
                
            v = J.T @ np.linalg.inv(J @ J.T + damp * np.eye(J.shape[0])) @ err_vec
            
            v_masked = np.zeros(self.model.nv)
            for jid in joint_ids:
                idx_v = self.model.joints[jid].idx_v
                v_masked[idx_v] = v[idx_v]

            q = pin.integrate(self.model, q, v_masked * dt)
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
            
        return q, False, final_err

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticIKCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
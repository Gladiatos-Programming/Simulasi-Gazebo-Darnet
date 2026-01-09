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

class DiagnosticIKCalculator(Node):
    def __init__(self):
        super().__init__('diagnostic_ik_calculator')
        
        # --- 1. LOAD MODEL ---
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

        # --- 2. CONFIG GROUPS ---
        self.target_config = {
            'RightArm': {
                'joints': ['Bahu Tangan Kanan', 'Lengan Kanan', 'Tangan Kanan'], 
                'ee_link': 'End_Effector_Tangan_Kanan_1',
                'topic': '/target_pose/right_arm',
                'solve_rotation': False  # Tangan cuma 3 DOF -> Posisi Only
            },
            'RightLeg': {
                'joints': ['Paha Kanan Putar','Paha Atas Kanan', 'Paha Bawah Kanan', 'Lutut Kanan', 'Kaki Kanan Atas', 'Kaki Kanan Bawah'], 
                'ee_link': 'End_Effector_Kaki_Kanan_1', 
                'topic': '/target_pose/right_leg',
                'solve_rotation': True   # Kaki 5 DOF -> Posisi + Orientasi (Tapak Kaki)
            },
            'LeftLeg': {
                'joints': ['Paha Kiri Putar','Paha Atas Kiri', 'Paha Bawah Kiri', 'Lutut Kiri', 'Kaki Kiri Atas', 'Kaki Kiri Bawah'], 
                'ee_link': 'End_Effector_Kaki_Kiri_1', 
                'topic': '/target_pose/left_leg',
                'solve_rotation': True   # Kaki 5 DOF -> Posisi + Orientasi (Tapak Kaki)
            }
        }

        # --- 3. PRE-PROCESS GROUPS ---
        self.groups_data = {} 
        self.all_monitored_joints = [] 

        for group_name, config in self.target_config.items():
            joint_names = config['joints']
            joint_ids = []
            valid_group = True

            for name in joint_names:
                if self.model.existJointName(name):
                    joint_ids.append(self.model.getJointId(name))
                    if name not in self.all_monitored_joints:
                        self.all_monitored_joints.append(name)
                else:
                    self.get_logger().error(f"❌ Joint '{name}' NOT FOUND!")
                    valid_group = False
            
            ee_name = config['ee_link']
            if self.model.existFrame(ee_name):
                ee_frame_id = self.model.getFrameId(ee_name)
            else:
                self.get_logger().error(f"❌ EE '{ee_name}' NOT FOUND!")
                ee_frame_id = -1
                valid_group = False

            if valid_group:
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
                mode_str = "6D (Pos+Rot)" if config['solve_rotation'] else "3D (Pos Only)"
                self.get_logger().info(f"✅ {group_name} Ready [{mode_str}]. Topic: {config['topic']}")

        # --- 4. ROS COMMS ---
        self.joint_state_received = False
        self.current_joint_states = {}
        
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.joint_cmd_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.q_current = pin.neutral(self.model)
        self.create_timer(2.0, self.check_connection)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🤖 WHOLE BODY IK (5DOF Supported)")
        self.get_logger().info("="*60 + "\n")

    def joint_state_callback(self, msg):
        self.joint_state_received = True
        for i, name in enumerate(msg.name):
            if name in self.all_monitored_joints:
                self.current_joint_states[name] = msg.position[i]

    def check_connection(self):
        if not self.joint_state_received:
            self.get_logger().warn("⚠️  WAITING FOR /joint_states...")

    def update_q_from_states(self):
        if not self.current_joint_states: return
        for name, pos in self.current_joint_states.items():
            if self.model.existJointName(name):
                jid = self.model.getJointId(name)
                idx_q = self.model.joints[jid].idx_q
                self.q_current[idx_q] = pos

    def callback_generic_target(self, msg, group_name):
        if group_name not in self.groups_data: return
        group_info = self.groups_data[group_name]
        
        # 1. Ambil Target Posisi & Orientasi
        target_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        # Buat Quaternion Pinocchio (x,y,z,w) -> Note: Pinocchio pakai (x,y,z,w) di constructor
        target_quat = pin.Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        target_SE3 = pin.SE3(target_quat.matrix(), target_pos)

        self.get_logger().info(f"\n🎯 TARGET {group_name}:")
        self.get_logger().info(f"   Pos: {target_pos}")
        if group_info['solve_rotation']:
            self.get_logger().info(f"   Rot (Quat): [{msg.orientation.x:.2f}, {msg.orientation.y:.2f}, {msg.orientation.z:.2f}, {msg.orientation.w:.2f}]")

        self.update_q_from_states()

        # 2. Hitung IK (Sekarang support Rotation)
        q_solusi, success, err = self.compute_ik(
            target_SE3, 
            group_info['joint_ids'], 
            group_info['ee_frame_id'],
            group_info['solve_rotation']
        )

        if success:
            self.get_logger().info(f"✅ IK CONVERGED (Err: {err:.4f})")
        else:
            self.get_logger().warn(f"⚠️  IK LIMIT (Err: {err:.4f})")
            self.analyze_failure(q_solusi, group_info['joint_names'], group_info['joint_ids'])

        self.send_joint_commands(q_solusi, group_info['joint_names'])

    def compute_ik(self, target_SE3, joint_ids, ee_frame_id, solve_rotation):
        q = self.q_current.copy()
        eps = 1e-3 
        max_iter = 2000
        dt = 0.1
        damp = 1e-3 # Damping sedikit dinaikkan untuk stabilitas 6D

        success = False
        final_err = 0.0

        for i in range(max_iter):
            pin.framesForwardKinematics(self.model, self.data, q)
            current_SE3 = self.data.oMf[ee_frame_id]
            
            # --- LOGIKA BARU: PILIH 3D ATAU 6D ---
            if solve_rotation:
                # ERROR 6 DIMENSI (Posisi + Rotasi)
                # actInv menghitung selisih transformasi
                error_se3 = current_SE3.actInv(target_SE3)
                # Log mapping mengubah SE3 error menjadi vector 6D (v, w)
                err_vec = pin.log(error_se3).vector 
            else:
                # ERROR 3 DIMENSI (Posisi Saja)
                err_vec = target_SE3.translation - current_SE3.translation

            final_err = np.linalg.norm(err_vec)
            if final_err < eps:
                return q, True, final_err

            # --- JACOBIAN ---
            J = pin.computeFrameJacobian(self.model, self.data, q, ee_frame_id, pin.ReferenceFrame.LOCAL)
            
            if not solve_rotation:
                # Kalau cuma posisi, ambil 3 baris teratas (Linear Velocity)
                # Dan convert ke WORLD frame orientation untuk translasi murni
                J = pin.computeFrameJacobian(self.model, self.data, q, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                J = J[:3, :] 
                
            # Damped Least Squares Solver
            v = J.T @ np.linalg.inv(J @ J.T + damp * np.eye(J.shape[0])) @ err_vec
            
            v_masked = np.zeros(self.model.nv)
            for jid in joint_ids:
                idx_v = self.model.joints[jid].idx_v
                v_masked[idx_v] = v[idx_v]

            q = pin.integrate(self.model, q, v_masked * dt)
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
            
        return q, False, final_err

    def analyze_failure(self, q, joint_names, joint_ids):
        self.get_logger().info("\n⚠️  JOINT STATUS:")
        for i, name in enumerate(joint_names):
            jid = joint_ids[i]
            idx_q = self.model.joints[jid].idx_q
            val = q[idx_q]
            min_lim = self.model.lowerPositionLimit[idx_q]
            max_lim = self.model.upperPositionLimit[idx_q]
            
            status = "OK"
            if np.isclose(val, min_lim, atol=0.02): status = "MIN"
            elif np.isclose(val, max_lim, atol=0.02): status = "MAX"
            self.get_logger().info(f"   {name}: {val:.2f} [{min_lim:.2f}|{max_lim:.2f}] -> {status}")

    def send_joint_commands(self, q_solusi, joint_names):
        try:
            target_values = []
            for j_name in joint_names:
                j_id = self.model.getJointId(j_name)
                idx_q = self.model.joints[j_id].idx_q
                target_values.append(float(q_solusi[idx_q]))

            msg = JointTrajectory()
            msg.joint_names = joint_names
            point = JointTrajectoryPoint()
            point.positions = target_values
            point.time_from_start = Duration(sec=0, nanosec=500000)
            msg.points.append(point)
            
            self.joint_cmd_pub.publish(msg)
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Send Error: {e}")
            return False

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
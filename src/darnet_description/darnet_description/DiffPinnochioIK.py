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

# IMPORT TF2 ROS
from tf2_ros import Buffer, TransformListener, TransformException

class DifferentialIKCalculator(Node):
    def __init__(self):
        super().__init__('diagnostic_ik_calculator')
        
        # --- 1. TF LISTENER (ODOM KE BASE) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.odom_frame = 'odom'
        self.base_frame = 'base_center'
        
        # --- 2. LOAD MODEL ---
        try:
            share_dir = get_package_share_directory('darnet_description')
            xacro_file = os.path.join(share_dir, 'urdf', 'darnet.xacro')
            doc = xacro.process_file(xacro_file)
            self.model = pin.buildModelFromXML(doc.toxml())
            self.data = self.model.createData()
            self.get_logger().info("✅ Pinocchio Model Loaded for Differential IK.")
        except Exception as e:
            self.get_logger().error(f"❌ Error loading URDF: {e}")
            raise e

        # --- 3. STATE MEMORY ---
        self.q_cmd = pin.neutral(self.model)
        self.global_joint_targets = {} 
        for i in range(self.model.nq):
            name = self.model.names[i]
            if name != "universe":
                self.global_joint_targets[name] = 0.0

        # --- KNEE CONSTRAINTS (Prioritas Tekuk Lutut) ---
        # Nilai sign menentukan arah "wajib" (misal: lutut kanan harus negatif, kiri positif)
        self.knee_config = {
            'Lutut Kanan': -1.0, 
            'Lutut Kiri':   1.0
        }
        self.knee_constraints = {}
        
        # Mapping dari nama joint lutut ke index array kecepatan (idx_v)
        for name, sign in self.knee_config.items():
            if self.model.existJointName(name):
                jid = self.model.getJointId(name)
                # Pastikan menyimpan idx_v (index velocity), bukan idx_q untuk diff-IK
                self.knee_constraints[self.model.joints[jid].idx_v] = sign
                self.get_logger().info(f"Knee Constraint applied to '{name}' with sign {sign}")

        # --- 4. CONFIG GROUPS ---
        self.target_config = {
            'RightArm': {'joints': ['Bahu Tangan Kanan', 'Lengan Kanan', 'Tangan Kanan'], 'ee_link': 'End_Effector_Tangan_Kanan_1', 'topic': '/target_pose/right_arm', 'solve_rotation': False},
            'RightLeg': {'joints': ['Paha Kanan Putar','Paha Atas Kanan', 'Paha Bawah Kanan', 'Lutut Kanan', 'Kaki Kanan Atas', 'Kaki Kanan Bawah'], 'ee_link': 'End_Effector_Kaki_Kanan_1', 'topic': '/target_pose/right_leg', 'solve_rotation': True},
            'LeftLeg': {'joints': ['Paha Kiri Putar','Paha Atas Kiri', 'Paha Bawah Kiri', 'Lutut Kiri', 'Kaki Kiri Atas', 'Kaki Kiri Bawah'], 'ee_link': 'End_Effector_Kaki_Kiri_1', 'topic': '/target_pose/left_leg', 'solve_rotation': True},
            'LeftArm': {'joints': ['Bahu Tangan Kiri', 'Lengan Kiri', 'Tangan Kiri'], 'ee_link': 'End_Effector_Tangan_Kiri_1', 'topic': '/target_pose/left_arm', 'solve_rotation': False}
        }

        self.groups_data = {} 
        self.all_monitored_joints = [] 
        self.active_targets = {}

        for group_name, config in self.target_config.items():
            joint_names = config['joints']
            joint_ids = []
            for name in joint_names:
                if self.model.existJointName(name):
                    joint_ids.append(self.model.getJointId(name))
                    if name not in self.all_monitored_joints:
                        self.all_monitored_joints.append(name)
            
            ee_name = config['ee_link']
            ee_frame_id = self.model.getFrameId(ee_name) if self.model.existFrame(ee_name) else -1

            self.groups_data[group_name] = {
                'joint_names': joint_names,
                'joint_ids': joint_ids,
                'ee_frame_id': ee_frame_id,
                'solve_rotation': config['solve_rotation']
            }
            self.create_subscription(Pose, config['topic'], lambda msg, g=group_name: self.callback_save_target(msg, g), 10)

        # --- 5. ROS COMMS ---
        self.current_joint_states = {}
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.joint_cmd_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.reset_service = self.create_service(Trigger, '/reset_ik_memory', self.callback_reset_memory)
        
        self.initialized = False
        self.init_timer = self.create_timer(1.0, self.initialize_targets_once)

        # --- 6. DIFFERENTIAL CONTROL LOOP ---
        self.control_rate = 50.0 
        self.dt = 1.0 / self.control_rate
        self.control_timer = self.create_timer(self.dt, self.control_loop)

        self.ik_gain = 10.0  
        self.damp = 1e-4    
        # Bobot untuk seberapa kuat lutut ditarik ke postur default
        self.knee_bias_weight = 0.5 

    def callback_reset_memory(self, request, response):
        self.update_q_cmd_from_real_sensors()
        self.active_targets.clear() 
        response.success = True
        response.message = "Differential IK Memory Synced!"
        return response

    def initialize_targets_once(self):
        if self.current_joint_states and not self.initialized:
            self.update_q_cmd_from_real_sensors()
            self.initialized = True
            self.init_timer.cancel()
            self.get_logger().info("✅ System Initialized. Control Loop Active.")

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.all_monitored_joints:
                self.current_joint_states[name] = msg.position[i]

    def update_q_cmd_from_real_sensors(self):
        for name, pos in self.current_joint_states.items():
            if self.model.existJointName(name):
                jid = self.model.getJointId(name)
                idx_q = self.model.joints[jid].idx_q
                self.q_cmd[idx_q] = pos
                self.global_joint_targets[name] = pos

    def callback_save_target(self, msg, group_name):
        self.active_targets[group_name] = msg

    def control_loop(self):
        if not self.initialized or not self.active_targets:
            return

        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.odom_frame, rclpy.time.Time())
        except TransformException:
            return

        tf_trans = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        tf_quat = pin.Quaternion(t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z)
        T_base_odom = pin.SE3(tf_quat.matrix(), tf_trans)

        v_full_robot = np.zeros(self.model.nv)
        pin.framesForwardKinematics(self.model, self.data, self.q_cmd)

        for group_name, pose_msg in self.active_targets.items():
            group_info = self.groups_data[group_name]
            ee_frame_id = group_info['ee_frame_id']
            solve_rotation = group_info['solve_rotation']

            target_pos = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
            target_quat = pin.Quaternion(pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z)
            T_odom_target = pin.SE3(target_quat.matrix(), target_pos)
            target_SE3 = T_base_odom.act(T_odom_target)

            current_SE3 = self.data.oMf[ee_frame_id]

            if solve_rotation:
                error_se3 = current_SE3.actInv(target_SE3)
                err_vec = pin.log(error_se3).vector 
            else:
                err_vec = target_SE3.translation - current_SE3.translation

            if np.linalg.norm(err_vec) < 1e-3:
                continue

            J = pin.computeFrameJacobian(self.model, self.data, self.q_cmd, ee_frame_id, pin.ReferenceFrame.LOCAL)
            if not solve_rotation:
                J = pin.computeFrameJacobian(self.model, self.data, self.q_cmd, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                J = J[:3, :] 

            # Kalkulasi Damped Pseudo-Inverse: J_pinv
            J_pinv = J.T @ np.linalg.inv(J @ J.T + self.damp * np.eye(J.shape[0]))
            
            # 1. Primary Task: Menuju ke target Cartesian
            v_primary = J_pinv @ err_vec

            # 2. Secondary Task: Null-Space Projection untuk postur lutut
            # v_nullspace = (I - J_pinv * J) * v_posture_bias
            v_posture_bias = np.zeros(self.model.nv)
            
            # Kita masukkan kecenderungan (bias) kecepatan agar lutut nekuk ke arah yang benar
            # Hanya diproses jika group ini melibatkan kaki (memiliki joint lutut)
            for jid in group_info['joint_ids']:
                idx_v = self.model.joints[jid].idx_v
                if idx_v in self.knee_constraints:
                    # Dorong kecepatan lutut sesuai dengan 'sign' yang diminta (ke depan)
                    # Semakin lurus lututnya (mendekati 0), dorongannya semakin kuat
                    current_angle = self.q_cmd[self.model.joints[jid].idx_q]
                    desired_sign = self.knee_constraints[idx_v]
                    
                    # Tambahkan bias kecepatan jika sudut lutut melawan arah yang seharusnya
                    # atau jika lutut terlalu lurus (bahaya singularity balik arah)
                    if (current_angle * desired_sign) < 0.1: 
                        v_posture_bias[idx_v] = desired_sign * self.knee_bias_weight
            
            # Kalkulasi proyektor Null-Space
            I_nv = np.eye(self.model.nv)
            null_space_projector = I_nv - (J_pinv @ J)
            
            # Gabungkan Primary Task dan Secondary Task (Postur)
            v_combined = v_primary + (null_space_projector @ v_posture_bias)
            
            # Masking: Terapkan kecepatan ke array full robot
            for jid in group_info['joint_ids']:
                idx_v = self.model.joints[jid].idx_v
                
                # --- EXTRA SAFETY: HARD CLAMPING ---
                # Kalau dari kalkulasi combined kecepatan lututnya malah bikin dia lurus ke arah yg salah,
                # kita nol-kan (atau kurangi) biar dia nggak nekuk ke belakang.
                if idx_v in self.knee_constraints:
                    desired_sign = self.knee_constraints[idx_v]
                    # Jika kecepatan mau membawa sendi ke arah berlawanan, kurangi drastis
                    if (v_combined[idx_v] * desired_sign) < 0:
                        v_combined[idx_v] *= 0.1 # Diredam 90%
                        
                v_full_robot[idx_v] = v_combined[idx_v]

        self.q_cmd = pin.integrate(self.model, self.q_cmd, v_full_robot * self.dt * self.ik_gain)
        
        # --- POSITIONAL HARD LIMITING UNTUK LUTUT ---
        # Ini step terakhir untuk menjamin 100% lutut gak pindah alam
        for idx_v, desired_sign in self.knee_constraints.items():
            # Cari idx_q yang bersesuaian dengan idx_v (asumsi joint 1 DOF)
            # Karena di Pinocchio idx_q bisa beda dari idx_v, kita iterasi sebentar
            idx_q = -1
            for j in self.model.joints:
                if j.idx_v == idx_v:
                    idx_q = j.idx_q
                    break
                    
            if idx_q != -1:
                # Jika diminta nekuk negatif (misal lutut kanan), maka max-nya harus 0.0 (atau sedikit lebih kecil)
                if desired_sign < 0:
                    if self.q_cmd[idx_q] > -0.05:
                        self.q_cmd[idx_q] = -0.05
                # Jika diminta nekuk positif (misal lutut kiri), maka min-nya harus 0.0
                elif desired_sign > 0:
                    if self.q_cmd[idx_q] < 0.05:
                        self.q_cmd[idx_q] = 0.05

        self.q_cmd = np.clip(self.q_cmd, self.model.lowerPositionLimit, self.model.upperPositionLimit)

        for name in self.all_monitored_joints:
            jid = self.model.getJointId(name)
            idx_q = self.model.joints[jid].idx_q
            self.global_joint_targets[name] = float(self.q_cmd[idx_q])

        self.publish_all_joints()

    def publish_all_joints(self):
        msg = JointTrajectory()
        msg.joint_names = self.all_monitored_joints
        
        point = JointTrajectoryPoint()
        point.positions = [self.global_joint_targets[name] for name in self.all_monitored_joints]
        
        ns_time = int((self.dt * 1.5) * 1e9)
        point.time_from_start = Duration(sec=0, nanosec=ns_time) 
        
        msg.points.append(point)
        self.joint_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialIKCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
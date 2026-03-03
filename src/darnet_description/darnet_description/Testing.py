#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
import time

class WalkingNode(Node):
    def __init__(self):
        super().__init__('walking_pattern_generator')
        
        # Publisher ke topik yang dibaca oleh script Differential IK
        self.pub_right = self.create_publisher(Pose, '/target_pose/right_leg', 10)
        self.pub_left = self.create_publisher(Pose, '/target_pose/left_leg', 10)
        self.sub_right = self.create_subscription(Pose, '/target_pose/right_leg', self.right_cb, 10)
        self.sub_left = self.create_subscription(Pose, '/target_pose/left_leg', self.left_cb, 10)
        self.default_quat = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}

        # Config
        self.T = 1.5
        self.stepheight = 0.06
        self.landrate = 0.30
        self.forward = 0.025
        self.z_threshold = 0.01
        self.stance_width = 0.15
        self.walk_phase = "IDLE"
        self.shift_x_leg = 0.06
        self.UVCcorrection = 0.0
        self.forward0_stance = 0.0
        self.forward0_swing = 0.0

        self.starting_position()

        # # Timer berjalan di 50 Hz (0.02 detik)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        
    def right_cb(self, msg):
        self.z_right = msg.position.z
        self.x_right = msg.position.x
        self.y_right = msg.position.y

    def left_cb(self, msg):
        self.z_left = msg.position.z
        self.x_left = msg.position.x
        self.y_left = msg.position.y

    def starting_position(self):
        pose_r = Pose()
        pose_l = Pose()

        pose_r.position.x = 0.055
        pose_l.position.x = -0.055
        toleransi = 0.001
        while abs(pose_l.position.x + (self.stance_width/2)) > toleransi or abs(pose_r.position.x - (self.stance_width/2)) > toleransi:
            pose_r.position.y = 0.0
            pose_l.position.y = 0.0
            pose_r.position.x += 0.0001 * (self.stance_width/2 - pose_r.position.x) 
            pose_l.position.x -= 0.0001 * (self.stance_width/2 + pose_l.position.x)
            pose_r.position.z = 0.007
            pose_l.position.z = 0.007

            pose_r.orientation.w = self.default_quat['w']
            pose_r.orientation.x = self.default_quat['x']
            pose_r.orientation.y = self.default_quat['y']
            pose_r.orientation.z = self.default_quat['z']

            pose_l.orientation.w = self.default_quat['w']
            pose_l.orientation.x = self.default_quat['x']
            pose_l.orientation.y = self.default_quat['y']
            pose_l.orientation.z = self.default_quat['z']

            self.pub_right.publish(pose_r)
            self.pub_left.publish(pose_l)
        print("starting position reached")

        # while pose_l.position.x != (self.stance_width/2) and pose_r.position.x != -(self.stance_width/2):

        #     pose_r.position.y = 0.0
        #     pose_l.position.y = 0.0
        #     pose_r.position.x = self.x_right 
        #     pose_l.position.x = self.x_right
        #     pose_r.position.z = 0.001
        #     pose_l.position.z = 0.001

        #     pose_r.orientation.w = self.default_quat['w']
        #     pose_r.orientation.x = self.default_quat['x']
        #     pose_r.orientation.y = self.default_quat['y']
        #     pose_r.orientation.z = self.default_quat['z']

        #     pose_l.orientation.w = self.default_quat['w']
        #     pose_l.orientation.x = self.default_quat['x']
        #     pose_l.orientation.y = self.default_quat['y']
        #     pose_l.orientation.z = self.default_quat['z']

        #     self.pub_right.publish(pose_r)
        #     self.pub_left.publish(pose_l)

    def timer_callback(self):

        right_contact = self.z_right <= self.z_threshold
        left_contact = self.z_left <= self.z_threshold

        if right_contact and left_contact:
            current_state = "DOUBLE"
            
        elif right_contact and not left_contact:
            current_state = "RIGHT"
            
        elif left_contact and not right_contact:
            current_state = "LEFT"
            
        else:
            current_state = "NONE"
        
        pose_r = Pose()
        pose_l = Pose()

        print(f"Current State: {current_state}")

        if self.walk_phase == "IDLE":
            self.rightnowposition_x = self.x_right
            self.rightnowposition_z = self.z_right
            self.leftnowposition_x = self.x_left
            self.leftnowposition_z = self.z_left
            self.rightnowposition_y = self.y_right
            self.leftnowposition_y = self.y_left
            
            self.walk_phase = "SHIFT_OUT"
            self.stance_leg = "RIGHT"         
            self.start_time = time.time()      


        # PENGAMAN POSISI (Setiap frame harus di-set awal dulu)
        if self.walk_phase != "IDLE":
            pose_r.position.x = self.rightnowposition_x
            pose_l.position.x = self.leftnowposition_x
            pose_r.position.z = self.rightnowposition_z
            pose_l.position.z = self.leftnowposition_z
            pose_r.position.y = self.rightnowposition_y
            pose_l.position.y = self.leftnowposition_y

        # =========================================================
        # FASE JALAN KONTINU (Gabungan Goyang & Angkat Kaki)
        # =========================================================
        if self.walk_phase == "SHIFT_OUT":  # Kita tetap pakai nama state ini
            t = time.time() - self.start_time

            # KONDISI RESET (1 Langkah Selesai Penuh)
            if t >= self.T:
                self.stance_leg = "LEFT" if self.stance_leg == "RIGHT" else "RIGHT"
                self.start_time = time.time()
                
                # Sumbu X tetap ngikutin posisi terakhir karena badannya emang geser
                self.rightnowposition_x = self.x_right
                self.leftnowposition_x = self.x_left
                
                # --- PERBAIKAN: Kunci sumbu Z selalu kembali ke lantai dasar ---
                self.rightnowposition_z = 0.007 
                self.leftnowposition_z = 0.007
                # ---------------------------------------------------------------

                if self.stance_leg == "RIGHT":
                    self.forward0_stance = self.y_right  # Stance di awal langkah
                    self.forward0_swing = self.y_left    # Swing di awal langkah
                elif self.stance_leg == "LEFT":
                    self.forward0_stance = self.y_left
                    self.forward0_swing = self.y_right
                
                # print(f"Step completed. New stance leg: {self.stance_leg}")
                t = 0.0

           # ---------------------------------------------------------
            # 1. PERHITUNGAN SUMBU X (Swaying / Goyang Badan)
            # Jalan penuh dari 0 -> Max (di T/2) -> 0 (di T)
            # ---------------------------------------------------------
            p_x = t / self.T  
            k = self.shift_x_leg * math.sin(math.pi * p_x)
            if self.stance_leg == "RIGHT":
                pose_r.position.x = self.rightnowposition_x - k
                pose_l.position.x = self.leftnowposition_x - (k/1.5)
            elif self.stance_leg == "LEFT":
                pose_l.position.x = self.leftnowposition_x + k
                pose_r.position.x = self.rightnowposition_x + (k/1.5)

            # ---------------------------------------------------------
            # 2. PERHITUNGAN SUMBU Z & Y (Menggunakan Rumus Lu)
            # ---------------------------------------------------------
            t_start_swing = self.T * self.landrate        
            t_end_swing = self.T * (1.0 - self.landrate)  

            # Ambil nilai yang udah di-snapshot tadi
            forward0 = self.forward0_stance
            target_stance = -(self.forward - self.UVCcorrection)
            target_swing = (self.forward - self.UVCcorrection)

            if t <= t_start_swing:
                # Fase Nunggu Awal (Double Support)
                htau = 0.0
                dy_stance = self.forward0_stance
                dy_swing = self.forward0_swing
                
            elif t >= t_end_swing:
                # Fase Nunggu Akhir (Double Support setelah mendarat)
                htau = 0.0
                dy_stance = -(self.forward - self.UVCcorrection)
                dy_swing = (self.forward - self.UVCcorrection)
                
            else:
                t_active = t - t_start_swing
                swing_duration = t_end_swing - t_start_swing
                
                up_duration = swing_duration * 0.40  
                down_duration = swing_duration * 0.60 

                if t_active < up_duration:
                    # FASE NAIK Z
                    p_z = t_active / up_duration 
                    ease = p_z * p_z * p_z * (p_z * (p_z * 6.0 - 15.0) + 10.0)
                    htau = self.stepheight * ease

                    # Fase langkah pertama
                    dy_stance = forward0 * (1.0 - t_active / up_duration)
                    dy_swing = self.forward0_swing * (1.0 - t_active / up_duration)

                    # dy_stance = forward0 * (1.0 - 2.0 * t_active/swing_duration)
                    # dy_swing = self.forward0_swing * (1.0 - 2.0 * t_active/swing_duration) * -1.0
                else:
                    # FASE TURUN Z
                    p_z = (t_active - up_duration) / down_duration 
                    ease = p_z * p_z * p_z * (p_z * (p_z * 6.0 - 15.0) + 10.0)
                    if p_z > 0.9:
                        ease = ease + (1.0 - ease) * 0.5 
                    htau = self.stepheight * (1.0 - ease)

                    # Fase langkah terakhir
                    progress_y = (t_active - up_duration) / down_duration
                    dy_stance = target_stance * progress_y
                    dy_swing = target_swing * progress_y

            # Terapkan hasil htau dan dy ke masing-masing kaki
            if self.stance_leg == "RIGHT":
                pose_r.position.y = dy_stance
                pose_l.position.y = dy_swing
                pose_l.position.z = self.leftnowposition_z + htau
            elif self.stance_leg == "LEFT":
                pose_l.position.y = dy_stance
                pose_r.position.y = dy_swing
                pose_r.position.z = self.rightnowposition_z + htau


        pose_r.orientation.w = self.default_quat['w']
        pose_r.orientation.x = self.default_quat['x']
        pose_r.orientation.y = self.default_quat['y']
        pose_r.orientation.z = self.default_quat['z']

        pose_l.orientation.w = self.default_quat['w']
        pose_l.orientation.x = self.default_quat['x']
        pose_l.orientation.y = self.default_quat['y']
        pose_l.orientation.z = self.default_quat['z']

        self.pub_right.publish(pose_r)
        self.pub_left.publish(pose_l)


def main(args=None):
    rclpy.init(args=args)
    node = WalkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
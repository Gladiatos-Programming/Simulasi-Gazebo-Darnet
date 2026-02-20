#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

class VisualTargetInteractive(Node):
    def __init__(self):
        super().__init__('visual_target_interactive')
        
        # Publisher untuk mengirim command ke IK Pinocchio
        self.ik_pub = self.create_publisher(Pose, '/target_pose/left_arm', 10)
        
        # Server Interactive Marker
        self.server = InteractiveMarkerServer(self, "target_controls")
        
        # Posisi Awal (Default)
        self.current_pose = Pose()
        self.current_pose.position.x = 0.0
        self.current_pose.position.y = 0.0
        self.current_pose.position.z = 0.0
        self.current_pose.orientation.w = 1.0

        # Buat Markernya
        self.create_interactive_marker()
        
        self.get_logger().info("Interactive Marker Server Ready! Silakan geser bola di RViz.")

    def create_interactive_marker(self):
        # 1. Setup Marker Utama
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "ik_target_marker"
        int_marker.scale = 0.2
        int_marker.pose = self.current_pose

        # 2. Visualisasi (Bola Merah)
        box_marker = Marker()
        box_marker.type = Marker.SPHERE
        box_marker.scale.x = 0.05
        box_marker.scale.y = 0.05
        box_marker.scale.z = 0.05
        box_marker.color.r = 1.0
        box_marker.color.g = 0.0
        box_marker.color.b = 0.0
        box_marker.color.a = 1.0

        # Kontrol Visual (Biar bisa dilihat)
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        int_marker.controls.append(box_control)

        # 3. Kontrol Gerakan (Panah X, Y, Z)
        self.add_axis_control(int_marker, True, False, False) # X Axis (Move)
        self.add_axis_control(int_marker, False, True, False) # Y Axis (Move)
        self.add_axis_control(int_marker, False, False, True) # Z Axis (Move)
        self.add_axis_control(int_marker, True, True, True)   # 3D Move (Sphere)

        # 4. Apply ke Server
        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def add_axis_control(self, int_marker, x, y, z):
        control = InteractiveMarkerControl()
        
        if x:
            control.orientation.w = 1.0
            control.orientation.x = 1.0
            control.orientation.y = 0.0
            control.orientation.z = 0.0
            control.name = "move_x"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control)
        if y:
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 1.0
            control.orientation.z = 0.0
            control.name = "move_y"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control)
        if z:
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 0.0
            control.orientation.z = 1.0
            control.name = "move_z"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control)
        if x and y and z: # Free move
             control.interaction_mode = InteractiveMarkerControl.MOVE_3D
             int_marker.controls.append(control)


    def process_feedback(self, feedback):
        # Callback saat marker digeser di RViz
        if feedback.event_type == feedback.POSE_UPDATE:
            # Update posisi internal
            self.current_pose = feedback.pose
            
            # Publish ke topic agar Pinocchio membacanya
            self.ik_pub.publish(feedback.pose)
            
            # Log posisi (opsional, biar ga spam bisa dihapus)
            # self.get_logger().info(f"Target Moved: x={feedback.pose.position.x:.2f}, y={feedback.pose.position.y:.2f}, z={feedback.pose.position.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = VisualTargetInteractive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
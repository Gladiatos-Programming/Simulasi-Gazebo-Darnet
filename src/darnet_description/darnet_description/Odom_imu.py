import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class OdomFromImuNode(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        # Broadcaster untuk TF (odom -> base_center)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publisher untuk topic /odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Subscriber untuk topic /imu (tipe Vector3 untuk Roll, Pitch, Yaw)
        self.imu_sub = self.create_subscription(
            Vector3, 
            '/imu', 
            self.imu_callback, 
            10
        )
        
        # Variabel posisi (Karena belum ada encoder roda, kita set 0 dulu)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        self.get_logger().info("Odometry node berjalan. Frame: odom -> base_center")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Konversi dari Euler Angles (Vector3) ke Quaternion"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def imu_callback(self, msg):
        # 1. Ambil data dari Vector3 (Asumsi format dalam DERAJAT)
        roll_deg = msg.x
        pitch_deg = msg.y
        yaw_deg = 0.0
        
        # 2. Konversi Derajat ke Radian WAJIB di ROS
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)
        
        current_time = self.get_clock().now().to_msg()
        
        # 3. Masukkan nilai radian ke fungsi konversi quaternion
        quat = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

        # 4. Publish TF (Transform dari odom ke base_center)
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_center'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        t.transform.rotation = quat
        
        self.tf_broadcaster.sendTransform(t)

        # 5. Publish pesan Odometry ke topic /odom
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_center'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation = quat
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFromImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
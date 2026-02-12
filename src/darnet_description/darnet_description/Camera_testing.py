#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Library Penerjemah ROS -> OpenCV
import cv2 # Library OpenCV
from ultralytics import YOLO # Library YOLO untuk deteksi objek
import os
from ament_index_python.packages import get_package_share_directory

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')

        # Ganti dengan nama topik kamera kamu!
        self.topic_name = '/camera/image_raw'

        # Membuat Subscriber
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.listener_callback,
            10)

        # Inisialisasi Bridge
        self.bridge = CvBridge()

        # Load YOLO model
        package_share = get_package_share_directory('darnet_description')
        model_path = os.path.join(package_share, 'darnet_description', 'best_n.pt')
        self.get_logger().info(f"🤖 Loading YOLO model from: {model_path}")
        self.model = YOLO(model_path)
        self.get_logger().info(f"✅ Model loaded successfully!")

        self.get_logger().info(f"📷 Menunggu gambar dari topik: {self.topic_name} ...")

    def listener_callback(self, msg):
        try:
            # === LANGKAH 1: TERJEMAHKAN (ROS -> OpenCV) ===
            # Kita ubah format ROS (msg) jadi format OpenCV (cv_image)
            # "bgr8" untuk OpenCV (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # === LANGKAH 2: JALANKAN DETEKSI YOLO ===
            # Run inference dengan YOLO model
            results = self.model(cv_image, verbose=False)

            # Gambar hasil deteksi pada frame
            annotated_frame = results[0].plot()

            # === LANGKAH 3: TAMPILKAN ===
            cv2.imshow("YOLO Detection - Kamera Robot", annotated_frame)

            # Wajib ada: Memberi waktu 1ms bagi OpenCV untuk menggambar window
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Gagal memproses gambar: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
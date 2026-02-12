#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Library Penerjemah ROS -> OpenCV
import cv2 # Library OpenCV

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
        
        self.get_logger().info(f"📷 Menunggu gambar dari topik: {self.topic_name} ...")

    def listener_callback(self, msg):
        try:
            # === LANGKAH 1: TERJEMAHKAN (ROS -> OpenCV) ===
            # Kita ubah format ROS (msg) jadi format OpenCV (cv_image)
            # "rgb8" artinya Red-Green-Blue (Format standar OpenCV)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            
            # === LANGKAH 2: OLAH GAMBAR (OPSIONAL) ===
            # Di sini kamu bisa nambahin kode deteksi, garis, dll.
            # Contoh: Kita ubah jadi hitam putih (Grayscale)
            # gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # === LANGKAH 3: TAMPILKAN ===
            cv2.imshow("Kamera Robot (Live)", cv_image)
            
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
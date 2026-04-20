import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class QRPublisher(Node):
    def __init__(self):
        super().__init__('qr_publisher')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        img = cv2.imread("/home/pepe/dual_tech_ws/src/qr_reader/qr.png")

        if img is None:
            self.get_logger().error(f"Nie mogę znaleźć obrazu: /home/pepe/dual_tech_ws/src/qr_reader/qr.png")
            return

        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')

        self.timer = self.create_timer(1.0, lambda: self.pub.publish(msg))


def main():
    rclpy.init()
    node = QRPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
#sudo apt install libzbar0
#pip install pyzbar


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
import cv2

class QRNode(Node):
    def __init__(self):
        super().__init__('qr_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(String, '/qr_data', 10)

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        small = cv2.resize(frame, (320, 240))
        decoded_objects = decode(small)

        if decoded_objects:
            for obj in decoded_objects:
                data = obj.data.decode('utf-8')

                self.publisher.publish(String(data=data))
                self.get_logger().info(f"QR: {data}")
        else:
            self.get_logger().debug("Brak QR")


def main(args=None):
    rclpy.init(args=args)
    node = QRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

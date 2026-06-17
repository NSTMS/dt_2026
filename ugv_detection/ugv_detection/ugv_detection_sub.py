import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from ugv_detection.topics import DETECTION_TOPIC
from dualtech_msgs.msg import Detection


class UGVDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('ugv_detection_subscriber')
        self.subscription = self.create_subscription(
            Detection,
            DETECTION_TOPIC,
            self.callback,
            10,
        )
        self.bridge = CvBridge()
        self.get_logger().info(f'Subskrybent UGV na {DETECTION_TOPIC}')

    def callback(self, msg: Detection):
        self.get_logger().info(
            f'Typ: {msg.object_type} | QR: {msg.qr_value}'
        )
        frame = self.bridge.compressed_imgmsg_to_cv2(msg.object_image, desired_encoding='bgr8')
        cv2.imshow(msg.object_type, frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = UGVDetectionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

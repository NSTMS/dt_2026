import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from uav_detection.topics import DETECTION_TOPIC
from dualtech_msgs.msg import Detection


class UAVDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('uav_detection_subscriber')
        self.subscription = self.create_subscription(
            Detection,
            DETECTION_TOPIC,
            self.callback,
            10,
        )
        self.bridge = CvBridge()
        self.get_logger().info(f'Subskrybent UAV na {DETECTION_TOPIC}')

    def callback(self, msg: Detection):
        gps = msg.gps_location
        self.get_logger().info(
            f'Typ: {msg.object_type} | '
            f'GPS: {gps.latitude:.6f}, {gps.longitude:.6f} | '
            f'QR: {msg.qr_value}'
        )
        frame = self.bridge.compressed_imgmsg_to_cv2(msg.object_image, desired_encoding='bgr8')
        cv2.imshow(msg.object_type, frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = UAVDetectionSubscriber()
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

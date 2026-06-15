import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from uav_detection.topics import UAV_DETECTION_TOPIC
from dualtech_msgs.msg import UavDetection


class UAVDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('uav_detection_subscriber')
        self.subscription = self.create_subscription(
            UavDetection,
            UAV_DETECTION_TOPIC,
            self.callback,
            10,
        )
        self.bridge = CvBridge()
        self.get_logger().info(f'Subskrybent UAV na {UAV_DETECTION_TOPIC}')

    def callback(self, msg: UavDetection):
        self.get_logger().info(
            f'[ID {msg.object_id}] {msg.object_type} | '
            f'GPS: {msg.latitude:.6f}, {msg.longitude:.6f} | '
            f'alt: {msg.altitude_baro:.1f}m | '
            f'rel: ({msg.relative_x:.1f}, {msg.relative_y:.1f}, {msg.relative_z:.1f}) m | '
            f'QR: {msg.qr_value}'
        )
        frame = self.bridge.imgmsg_to_cv2(msg.object_image, desired_encoding='bgr8')
        cv2.imshow(f'{msg.object_type} #{msg.object_id}', frame)
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

import rclpy
from cv_bridge import CvBridge
from dualtech_detection.qr_decoder import decode_qr
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

CAMERA_IMAGE_TOPIC = '/camera/image_raw'
QR_DATA_TOPIC = '/qr_data'


class QRNode(Node):
    def __init__(self):
        super().__init__('qr_node')
        self._bridge = CvBridge()
        self._last_qr = ''

        self.create_subscription(Image, CAMERA_IMAGE_TOPIC, self._callback, 10)
        self._publisher = self.create_publisher(String, QR_DATA_TOPIC, 10)
        self.get_logger().info(f'QR reader: {CAMERA_IMAGE_TOPIC} → {QR_DATA_TOPIC}')

    def _callback(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        qr = decode_qr(frame)
        if not qr or qr == self._last_qr:
            return

        self._last_qr = qr
        self._publisher.publish(String(data=qr))
        self.get_logger().info(f'QR: {qr}')


def main(args=None):
    rclpy.init(args=args)
    node = QRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

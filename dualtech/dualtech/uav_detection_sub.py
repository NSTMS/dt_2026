import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from dualtech_msgs.msg import Detection
import cv2


class UAVDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')
        self.subscription = self.create_subscription(
            Detection,
            '/yolo/detections',
            self.callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Subskrybent detekcji uruchomiony")

    def callback(self, msg: Detection):
        self.get_logger().info(
            f"[ID {msg.object_id}] Typ: {msg.object_type}"
        )

        # Opcjonalnie: podgląd przyciętego obrazu
        frame = self.bridge.imgmsg_to_cv2(msg.object_image, desired_encoding="bgr8")
        cv2.imshow(f"{msg.object_type} #{msg.object_id}", frame)
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

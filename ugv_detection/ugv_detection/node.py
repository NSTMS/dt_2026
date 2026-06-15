import os
import threading
import time

import rclpy
from cv_bridge import CvBridge
from dualtech_detection.filter import SameDetectionFilter
from dualtech_detection.pipeline import DetectionLoop, GStreamerFrameStream, YoloDetector
from dualtech_detection.qr_decoder import decode_qr
from dualtech_detection.types import DetectionCandidate
from dualtech_msgs.msg import UgvDetection
from rclpy.node import Node
from ugv_detection.topics import UGV_DETECTION_TOPIC

TARGET_FPS = float(os.getenv('TARGET_FPS', '12'))
CAMERA_WIDTH = int(os.getenv('CAMERA_WIDTH', '640'))
CAMERA_HEIGHT = int(os.getenv('CAMERA_HEIGHT', '480'))
DETECTION_WINDOW_SEC = float(os.getenv('DETECTION_WINDOW_SEC', '10'))
DETECTION_MIN_COUNT = int(os.getenv('DETECTION_MIN_COUNT', '3'))
YOLO_CONFIDENCE = float(os.getenv('YOLO_CONFIDENCE', '0.5'))

GST_PIPELINE_CSI = (
    'libcamerasrc ! '
    'video/x-raw,format=NV12 ! '
    'videoconvert ! '
    'videoscale ! '
    f'video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},format=BGR ! '
    'appsink drop=true max-buffers=1'
)


class UgvDetectionNode(Node):
    def __init__(self):
        super().__init__('ugv_detection_node')

        self._publisher = self.create_publisher(UgvDetection, UGV_DETECTION_TOPIC, 10)
        self._bridge = CvBridge()
        self._filter = SameDetectionFilter(DETECTION_WINDOW_SEC, DETECTION_MIN_COUNT)
        self._object_id = 0

        self._detector = YoloDetector('yolov8n_ground.pt', YOLO_CONFIDENCE)
        self._stream = GStreamerFrameStream(
            GST_PIPELINE_CSI,
            logger=lambda msg: self.get_logger().error(msg),
        )

        self.get_logger().info(
            f'UGV: {UGV_DETECTION_TOPIC}, kamera {CAMERA_WIDTH}x{CAMERA_HEIGHT}, '
            f'filtr {DETECTION_MIN_COUNT}x/{DETECTION_WINDOW_SEC}s'
        )

    def start(self) -> None:
        self._stream.start()
        loop = DetectionLoop(
            self._stream,
            self._detector,
            self._on_frame,
            TARGET_FPS,
        )
        threading.Thread(target=loop.run, daemon=True).start()

    def _on_frame(self, frame, candidates: list[DetectionCandidate], annotated) -> None:
        if not rclpy.ok():
            return

        qr = decode_qr(frame)
        if qr:
            self.get_logger().info(f'QR: {qr}')

        if not candidates:
            return

        now = time.time()
        for candidate in candidates:
            if not self._filter.should_publish(candidate.class_name, now):
                self.get_logger().debug(
                    f'[{candidate.class_name}] oczekiwanie na {DETECTION_MIN_COUNT} trafienia'
                )
                continue

            self._publish_confirmed(candidate, annotated, qr)

    def _publish_confirmed(self, candidate: DetectionCandidate, annotated, qr: str) -> None:
        msg = UgvDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ugv_camera'
        msg.object_id = self._object_id
        msg.object_type = candidate.class_name
        msg.object_image = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        msg.qr_value = qr

        self._publisher.publish(msg)
        self._object_id += 1
        self.get_logger().info(f'Publikacja [{msg.object_type}] id={msg.object_id}')


def main(args=None):
    rclpy.init(args=args)
    node = UgvDetectionNode()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

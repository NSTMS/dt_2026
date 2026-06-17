import os
import threading
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from dualtech_detection.pipeline import DetectionLoop, GStreamerFrameStream, YoloDetector
from dualtech_detection.qr_decoder import decode_qr
from dualtech_detection.types import DetectionCandidate
from dualtech_msgs.msg import UgvDetection
from rclpy.node import Node
from sensor_msgs.msg import Image
from ugv_detection.topics import UGV_DETECTION_TOPIC

CAMERA_IMAGE_TOPIC = '/camera/image_raw'

TARGET_FPS = float(os.getenv('TARGET_FPS', '12'))
CAMERA_WIDTH = int(os.getenv('CAMERA_WIDTH', '1280'))
CAMERA_HEIGHT = int(os.getenv('CAMERA_HEIGHT', '720'))
CAMERA_SOURCE = os.getenv('CAMERA_SOURCE', 'csi').lower()
UDP_PORT = int(os.getenv('UDP_PORT', '5600'))
STREAM_HOST = os.getenv('STREAM_HOST', '')
STREAM_PORT = int(os.getenv('STREAM_PORT', '5600'))
YOLO_CONFIDENCE = float(os.getenv('YOLO_CONFIDENCE', '0.65'))
VOTING_WINDOW_SIZE = int(os.getenv('VOTING_WINDOW_SIZE', '10'))
VOTING_MIN_HITS = int(os.getenv('VOTING_MIN_HITS', '6'))
CLASS_WHITELIST = {
    name.strip() for name in os.getenv('CLASS_WHITELIST', '').split(',') if name.strip()
}
CLASS_BLACKLIST = {
    name.strip() for name in os.getenv('CLASS_BLACKLIST', '').split(',') if name.strip()
}
YOLO_MODEL = os.getenv(
    'YOLO_MODEL',
    os.path.join(get_package_share_directory('ugv_detection'), 'yolov8n_ground.pt'),
)

_CSI_SOURCE = (
    f'libcamerasrc ! '
    f'video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},framerate=30/1,format=NV12 ! '
    f'videoflip method=rotate-180 ! '
)
_APPSINK = (
    f'videoconvert ! videoscale ! '
    f'video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},format=BGR ! '
    f'appsink drop=true max-buffers=1'
)
_STREAM_BRANCH = (
    f'queue ! videoconvert ! video/x-raw,format=I420 ! '
    f'x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 key-int-max=30 ! '
    f'h264parse ! rtph264pay config-interval=1 pt=96 ! '
    f'multiudpsink clients={STREAM_HOST}:{STREAM_PORT} sync=false'
) if STREAM_HOST else None

if _STREAM_BRANCH:
    GST_PIPELINE_CSI = (
        _CSI_SOURCE +
        f'tee name=t '
        f't. ! queue ! {_APPSINK} '
        f't. ! {_STREAM_BRANCH}'
    )
else:
    GST_PIPELINE_CSI = _CSI_SOURCE + _APPSINK

GST_PIPELINE_UDP = (
    f'udpsrc port={UDP_PORT} ! '
    'application/x-rtp,encoding-name=H264,payload=96 ! '
    'rtph264depay ! h264parse ! avdec_h264 ! '
    'videoconvert ! '
    f'videoscale ! video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},format=BGR ! '
    'appsink drop=true max-buffers=1 sync=false'
)


class UgvDetectionNode(Node):
    def __init__(self):
        super().__init__('ugv_detection_node')

        self._publisher = self.create_publisher(UgvDetection, UGV_DETECTION_TOPIC, 10)
        self._image_publisher = self.create_publisher(Image, CAMERA_IMAGE_TOPIC, 10)
        self._bridge = CvBridge()
        self._object_id = 0
        self._recent_high_conf_classes: deque[str] = deque(maxlen=VOTING_WINDOW_SIZE)

        self._detector = YoloDetector(YOLO_MODEL, YOLO_CONFIDENCE)
        stream_pipeline = GST_PIPELINE_CSI if CAMERA_SOURCE == 'csi' else GST_PIPELINE_UDP
        self._stream = GStreamerFrameStream(
            stream_pipeline,
            logger=lambda msg: self.get_logger().error(msg),
        )

        self.get_logger().info(
            f'UGV: {UGV_DETECTION_TOPIC}, kamera {CAMERA_WIDTH}x{CAMERA_HEIGHT}, '
            f'publikacja gdy >= {VOTING_MIN_HITS}/{VOTING_WINDOW_SIZE} '
            f'detekcji (conf>={YOLO_CONFIDENCE:.2f})'
        )
        self.get_logger().info(f'Źródło kamery: {CAMERA_SOURCE} (UDP:{UDP_PORT})')
        if STREAM_HOST:
            self.get_logger().info(f'Streaming RTP → {STREAM_HOST}:{STREAM_PORT}')
        else:
            self.get_logger().info('Streaming RTP wyłączony (ustaw STREAM_HOST)')
        self.get_logger().info(f'Model YOLO: {YOLO_MODEL}')
        self.get_logger().info(f'Klasy modelu: {self._detector.class_names}')
        if CLASS_WHITELIST:
            self.get_logger().info(f'CLASS_WHITELIST={sorted(CLASS_WHITELIST)}')
        if CLASS_BLACKLIST:
            self.get_logger().info(f'CLASS_BLACKLIST={sorted(CLASS_BLACKLIST)}')

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

        image_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'ugv_camera'
        self._image_publisher.publish(image_msg)

        qr = decode_qr(frame)
        if qr:
            self.get_logger().info(f'QR: {qr}')

        if not candidates:
            self._recent_high_conf_classes.clear()
            return

        candidates = self._filter_classes(candidates)
        if not candidates:
            self._recent_high_conf_classes.clear()
            return

        # Dodatkowa ochrona: niezależnie od filtra YOLO odrzuć detekcje poniżej progu.
        candidates = [c for c in candidates if c.confidence >= YOLO_CONFIDENCE]
        if not candidates:
            self._recent_high_conf_classes.clear()
            return

        best_candidate = max(candidates, key=lambda c: c.confidence)
        voting_confirmed, class_hits = self._update_voting(best_candidate.class_name)
        self.get_logger().info(
            f'Wykryto [{best_candidate.class_name}] conf={best_candidate.confidence:.2f} '
            f'votes={class_hits}/{len(self._recent_high_conf_classes)}'
        )
        if voting_confirmed:
            self._publish_confirmed(best_candidate, annotated, qr)
            self._recent_high_conf_classes.clear()

    def _filter_classes(self, candidates: list[DetectionCandidate]) -> list[DetectionCandidate]:
        filtered = candidates
        if CLASS_WHITELIST:
            filtered = [c for c in filtered if c.class_name in CLASS_WHITELIST]
        if CLASS_BLACKLIST:
            filtered = [c for c in filtered if c.class_name not in CLASS_BLACKLIST]
        return filtered

    def _update_voting(self, class_name: str) -> tuple[bool, int]:
        self._recent_high_conf_classes.append(class_name)
        class_hits = sum(1 for name in self._recent_high_conf_classes if name == class_name)
        return class_hits >= VOTING_MIN_HITS, class_hits

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

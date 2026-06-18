import os
import threading
import time
from dataclasses import replace

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from dualtech_detection.pipeline import GStreamerFrameStream, YoloDetector
from dualtech_detection.qr_decoder import decode_qr
from dualtech_detection.types import DetectionCandidate
from dualtech_msgs.msg import Detection
from rclpy.node import Node
from ugv_detection.topics import DETECTION_TOPIC

TARGET_FPS = float(os.getenv('TARGET_FPS', '12'))
CAMERA_WIDTH = int(os.getenv('CAMERA_WIDTH', '1280'))
CAMERA_HEIGHT = int(os.getenv('CAMERA_HEIGHT', '720'))
CAMERA_SOURCE = os.getenv('CAMERA_SOURCE', 'csi').lower()
UDP_PORT = int(os.getenv('UDP_PORT', '5600'))
STREAM_HOST = os.getenv('STREAM_HOST', '')
STREAM_PORT = int(os.getenv('STREAM_PORT', '5600'))
YOLO_CONFIDENCE = float(os.getenv('YOLO_CONFIDENCE', '0.65'))
QR_CONFIRM_COUNT = int(os.getenv('QR_CONFIRM_COUNT', '3'))
CLASS_WHITELIST = {
    name.strip() for name in os.getenv('CLASS_WHITELIST', '').split(',') if name.strip()
}
CLASS_BLACKLIST = {
    name.strip() for name in os.getenv('CLASS_BLACKLIST', '').split(',') if name.strip()
}
_DEFAULT_CLASS_NAME_MAP = {
    'tico': 'maulch',
    'polonez': 'polonez',
    'lambo': 'ferrari',
    'bus': 'autobus',
    'tir': 'tir',
    'czolg_zielony': 'T-90',
    'czolg_bialy': 'T-62',
    'wyrzutnia': 'pansir',
    'humvee': 'humvee',
    'radar': 'radar',
}


def _parse_class_name_map(raw: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for pair in raw.split(','):
        pair = pair.strip()
        if not pair or ':' not in pair:
            continue
        old_name, new_name = pair.split(':', 1)
        old_name = old_name.strip()
        new_name = new_name.strip()
        if old_name and new_name:
            result[old_name] = new_name
    return result


CLASS_NAME_MAP = dict(_DEFAULT_CLASS_NAME_MAP)
CLASS_NAME_MAP.update(_parse_class_name_map(os.getenv('CLASS_NAME_MAP', '')))
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

        self._publisher = self.create_publisher(Detection, DETECTION_TOPIC, 10)
        self._bridge = CvBridge()
        self._object_id = 0
        self._qr_tracker: dict[str, list] = {}
        self._qr_blacklist: set[str] = set()

        self._detector = YoloDetector(YOLO_MODEL, YOLO_CONFIDENCE)
        stream_pipeline = GST_PIPELINE_CSI if CAMERA_SOURCE == 'csi' else GST_PIPELINE_UDP
        self._stream = GStreamerFrameStream(
            stream_pipeline,
            logger=lambda msg: self.get_logger().error(msg),
        )

        self.get_logger().info(
            f'UGV: {DETECTION_TOPIC}, kamera {CAMERA_WIDTH}x{CAMERA_HEIGHT}, '
            f'publikacja gdy ta sama klasa per QR {QR_CONFIRM_COUNT}x '
            f'(conf>={YOLO_CONFIDENCE:.2f})'
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
        self.get_logger().info(f'CLASS_NAME_MAP={CLASS_NAME_MAP}')

    def start(self) -> None:
        self._stream.start()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        frame_interval = 1.0 / TARGET_FPS
        last_time = 0.0
        while rclpy.ok():
            frame = self._stream.read(timeout=1.0)
            if frame is None:
                continue

            now = time.time()
            if now - last_time < frame_interval:
                continue
            last_time = now

            self._process_frame(frame)

    def _process_frame(self, frame) -> None:
        qr = decode_qr(frame)
        if not qr or qr in self._qr_blacklist:
            return

        candidates, annotated = self._detector.detect(frame)
        candidates = self._filter_classes(candidates)
        candidates = self._remap_classes(candidates)
        candidates = [c for c in candidates if c.confidence >= YOLO_CONFIDENCE]
        if not candidates:
            return

        best_candidate = max(candidates, key=lambda c: c.confidence)
        self._update_qr_tracker(qr, best_candidate, annotated, candidates)

    def _filter_classes(self, candidates: list[DetectionCandidate]) -> list[DetectionCandidate]:
        filtered = candidates
        if CLASS_WHITELIST:
            filtered = [c for c in filtered if c.class_name in CLASS_WHITELIST]
        if CLASS_BLACKLIST:
            filtered = [c for c in filtered if c.class_name not in CLASS_BLACKLIST]
        return filtered

    def _remap_classes(self, candidates: list[DetectionCandidate]) -> list[DetectionCandidate]:
        if not CLASS_NAME_MAP:
            return candidates
        return [
            replace(c, class_name=CLASS_NAME_MAP.get(c.class_name, c.class_name))
            for c in candidates
        ]

    def _update_qr_tracker(
        self,
        qr: str,
        candidate: DetectionCandidate,
        annotated,
        candidates: list[DetectionCandidate],
    ) -> None:
        detections_label = ', '.join(
            f'{c.class_name} {c.confidence:.2f}' for c in candidates
        )
        entry = self._qr_tracker.get(qr)
        if entry is None or entry[0] != candidate.class_name:
            self._qr_tracker[qr] = [candidate.class_name, 1]
            self.get_logger().info(
                f'QR {qr}: {detections_label} | '
                f'[{candidate.class_name}] count=1/{QR_CONFIRM_COUNT}'
            )
            return

        entry[1] += 1
        self.get_logger().info(
            f'QR {qr}: {detections_label} | '
            f'[{candidate.class_name}] count={entry[1]}/{QR_CONFIRM_COUNT}'
        )
        if entry[1] >= QR_CONFIRM_COUNT:
            self._publish_confirmed(candidate, annotated, qr)
            self._qr_blacklist.add(qr)
            del self._qr_tracker[qr]

    def _publish_confirmed(self, candidate: DetectionCandidate, annotated, qr: str) -> None:
        msg = Detection()
        msg.object_type = candidate.class_name
        image_msg = self._bridge.cv2_to_compressed_imgmsg(annotated)
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'ugv_camera'
        msg.object_image = image_msg
        msg.qr_value = qr

        self._publisher.publish(msg)
        self._object_id += 1
        self.get_logger().info(f'Publikacja [{msg.object_type}] id={self._object_id}')


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

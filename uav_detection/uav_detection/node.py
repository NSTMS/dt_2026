import os
import threading
import time
from dataclasses import replace

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from dualtech_detection.geolocation import CameraModel, estimate_ground_offset
from dualtech_detection.pipeline import GStreamerFrameStream, YoloDetector
from dualtech_detection.qr_decoder import decode_qr_all
from dualtech_detection.qr_proximity_aggregator import QrProximityAggregator
from dualtech_detection.types import DetectionCandidate
from dualtech_msgs.msg import UavDetection
from rclpy.node import Node
from std_msgs.msg import Bool

from uav_detection.telemetry import MavrosTelemetry
from uav_detection.topics import UAV_DETECTION_TOPIC

TARGET_FPS = float(os.getenv('TARGET_FPS', '15'))
CAMERA_WIDTH = int(os.getenv('CAMERA_WIDTH', '1280'))
CAMERA_HEIGHT = int(os.getenv('CAMERA_HEIGHT', '720'))
CAMERA_SOURCE = os.getenv('CAMERA_SOURCE', 'csi').lower()
UDP_PORT = int(os.getenv('UDP_PORT', '5000'))
STREAM_HOST = os.getenv('STREAM_HOST', '')
STREAM_PORT = int(os.getenv('STREAM_PORT', '5600'))
TRIGGER_DROP_ON_QR = os.getenv('TRIGGER_DROP_ON_QR', 'true').lower() == 'true'
QR_PROXIMITY_RADIUS_PX = float(os.getenv('QR_PROXIMITY_RADIUS_PX', '400'))
CONFIDENCE_SUM_THRESHOLD = float(os.getenv('CONFIDENCE_SUM_THRESHOLD', '1.3'))
CAMERA_H_FOV_DEG = float(os.getenv('CAMERA_H_FOV_DEG', '70'))
CAMERA_V_FOV_DEG = float(os.getenv('CAMERA_V_FOV_DEG', '50'))
YOLO_CONFIDENCE = float(os.getenv('YOLO_CONFIDENCE', '0.5'))
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
    os.path.join(get_package_share_directory('uav_detection'), 'yolov8n_aerial.pt'),
)

_CSI_SOURCE = (
    f'libcamerasrc ! '
    f'video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},framerate=30/1,format=NV12 ! '
    f'videoflip method=rotate-180 ! '
)
_UDP_SOURCE = (
    f'udpsrc port={UDP_PORT} ! '
    'application/x-rtp,encoding-name=H264,payload=96 ! '
    'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! '
)
_APPSINK = (
    f'videoconvert ! videoscale ! '
    f'video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},format=BGR ! '
    f'appsink drop=true max-buffers=1 sync=false'
)
_STREAM_BRANCH = (
    f'queue ! videoconvert ! video/x-raw,format=I420 ! '
    f'x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 key-int-max=30 ! '
    f'h264parse ! rtph264pay config-interval=1 pt=96 ! '
    f'multiudpsink clients={STREAM_HOST}:{STREAM_PORT} sync=false'
) if STREAM_HOST else None


def _build_pipeline(source: str) -> str:
    if _STREAM_BRANCH:
        return (
            source +
            f'tee name=t '
            f't. ! queue ! {_APPSINK} '
            f't. ! {_STREAM_BRANCH}'
        )
    return source + _APPSINK


GST_PIPELINE_CSI = _build_pipeline(_CSI_SOURCE)
GST_PIPELINE_UDP = _build_pipeline(_UDP_SOURCE)


class UavDetectionNode(Node):
    def __init__(self):
        super().__init__('uav_detection_node')

        self._publisher = self.create_publisher(UavDetection, UAV_DETECTION_TOPIC, 10)
        self._drop_trigger_pub = self.create_publisher(Bool, '/servo/trigger', 10)
        self._bridge = CvBridge()
        self._telemetry = MavrosTelemetry(self)
        self._aggregator = QrProximityAggregator(
            QR_PROXIMITY_RADIUS_PX,
            CONFIDENCE_SUM_THRESHOLD,
        )
        self._object_id = 0

        self._detector = YoloDetector(YOLO_MODEL, YOLO_CONFIDENCE)
        stream_pipeline = GST_PIPELINE_CSI if CAMERA_SOURCE == 'csi' else GST_PIPELINE_UDP
        self._stream = GStreamerFrameStream(
            stream_pipeline,
            logger=lambda msg: self.get_logger().error(msg),
        )

        self.get_logger().info(
            f'UAV: {UAV_DETECTION_TOPIC}, '
            f'QR proximity {QR_PROXIMITY_RADIUS_PX:.0f}px, '
            f'suma conf >= {CONFIDENCE_SUM_THRESHOLD:.1f} (conf>={YOLO_CONFIDENCE:.2f})'
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

    def destroy_node(self):
        self._telemetry.close()
        super().destroy_node()

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
        qrs = decode_qr_all(frame)
        if not qrs:
            return

        if TRIGGER_DROP_ON_QR:
            for qr in qrs:
                self.get_logger().info(f'QR: {qr.text} → /servo/trigger')
                self._drop_trigger_pub.publish(Bool(data=True))

        active_qr = next(
            (qr for qr in qrs if not self._aggregator.is_blacklisted(qr.text)),
            None,
        )
        if active_qr is None:
            return

        candidates, annotated = self._detector.detect(frame)
        candidates = self._filter_classes(candidates)
        candidates = self._remap_classes(candidates)
        if not candidates:
            return

        detections_label = ', '.join(
            f'{c.class_name} {c.confidence:.2f}' for c in candidates
        )
        self.get_logger().info(f'QR {active_qr.text}: {detections_label}')

        events = self._aggregator.add_detections(
            active_qr.text,
            active_qr.center,
            candidates,
        )

        camera = CameraModel(
            width=frame.shape[1],
            height=frame.shape[0],
            h_fov_deg=CAMERA_H_FOV_DEG,
            v_fov_deg=CAMERA_V_FOV_DEG,
        )
        telemetry = self._telemetry.snapshot()
        for event in events:
            self._publish_confirmed(
                event.candidate,
                annotated,
                event.qr_text,
                camera,
                telemetry,
            )
            self.get_logger().info(
                f'QR {event.qr_text}: [{event.candidate.class_name}] '
                f'suma conf={event.confidence_sum:.2f} → publikacja, blacklist'
            )

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

    def _publish_confirmed(
        self,
        candidate: DetectionCandidate,
        annotated,
        qr: str,
        camera: CameraModel,
        telemetry,
    ) -> None:
        cx, cy = candidate.bbox_center
        relative = estimate_ground_offset(cx, cy, telemetry.altitude_baro, camera)

        msg = UavDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'uav_camera'
        msg.object_id = self._object_id
        msg.object_type = candidate.class_name
        msg.object_image = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        msg.qr_value = qr
        msg.latitude = telemetry.latitude
        msg.longitude = telemetry.longitude
        msg.altitude_gps = telemetry.altitude_gps
        msg.altitude_baro = telemetry.altitude_baro
        msg.relative_x = relative.x
        msg.relative_y = relative.y
        msg.relative_z = relative.z

        self._publisher.publish(msg)
        self._object_id += 1

        self.get_logger().info(
            f'Publikacja [{msg.object_type}] id={msg.object_id} | '
            f'rel=({relative.x:.1f}, {relative.y:.1f}, {relative.z:.1f}) m | '
            f'alt={telemetry.altitude_baro:.1f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = UavDetectionNode()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

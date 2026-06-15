import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from dualtech_detection.filter import SameDetectionFilter
from dualtech_detection.geolocation import CameraModel, estimate_ground_offset
from dualtech_detection.pipeline import DetectionLoop, GStreamerFrameStream, YoloDetector
from dualtech_detection.qr_decoder import decode_qr
from dualtech_detection.types import DetectionCandidate
from dualtech_msgs.msg import UavDetection
from rclpy.node import Node
from std_msgs.msg import Bool

from uav_detection.telemetry import MavrosTelemetry
from uav_detection.topics import UAV_DETECTION_TOPIC

TARGET_FPS = float(os.getenv('TARGET_FPS', '15'))
UDP_PORT = int(os.getenv('UDP_PORT', '5000'))
TRIGGER_DROP_ON_QR = os.getenv('TRIGGER_DROP_ON_QR', 'true').lower() == 'true'
DETECTION_WINDOW_SEC = float(os.getenv('DETECTION_WINDOW_SEC', '10'))
DETECTION_MIN_COUNT = int(os.getenv('DETECTION_MIN_COUNT', '3'))
CAMERA_H_FOV_DEG = float(os.getenv('CAMERA_H_FOV_DEG', '70'))
CAMERA_V_FOV_DEG = float(os.getenv('CAMERA_V_FOV_DEG', '50'))
YOLO_CONFIDENCE = float(os.getenv('YOLO_CONFIDENCE', '0.5'))
YOLO_MODEL = os.getenv(
    'YOLO_MODEL',
    os.path.join(get_package_share_directory('uav_detection'), 'yolov8n_aerial.pt'),
)

GST_PIPELINE_UDP = (
    f'udpsrc port={UDP_PORT} ! '
    'application/x-rtp,encoding-name=H264,payload=96 ! '
    'rtph264depay ! h264parse ! avdec_h264 ! '
    'videoconvert ! appsink drop=true max-buffers=1 sync=false'
)


class UavDetectionNode(Node):
    def __init__(self):
        super().__init__('uav_detection_node')

        self._publisher = self.create_publisher(UavDetection, UAV_DETECTION_TOPIC, 10)
        self._drop_trigger_pub = self.create_publisher(Bool, '/servo/trigger', 10)
        self._bridge = CvBridge()
        self._telemetry = MavrosTelemetry(self)
        self._filter = SameDetectionFilter(DETECTION_WINDOW_SEC, DETECTION_MIN_COUNT)
        self._object_id = 0

        self._detector = YoloDetector(YOLO_MODEL, YOLO_CONFIDENCE)
        self._stream = GStreamerFrameStream(
            GST_PIPELINE_UDP,
            logger=lambda msg: self.get_logger().error(msg),
        )

        self.get_logger().info(
            f'UAV: {UAV_DETECTION_TOPIC}, UDP:{UDP_PORT}, '
            f'filtr {DETECTION_MIN_COUNT}x/{DETECTION_WINDOW_SEC}s'
        )

    def destroy_node(self):
        self._telemetry.close()
        super().destroy_node()

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
        if qr and TRIGGER_DROP_ON_QR:
            self.get_logger().info(f'QR: {qr} → /servo/trigger')
            trigger = Bool()
            trigger.data = True
            self._drop_trigger_pub.publish(trigger)

        if not candidates:
            return

        now = time.time()
        camera = CameraModel(
            width=frame.shape[1],
            height=frame.shape[0],
            h_fov_deg=CAMERA_H_FOV_DEG,
            v_fov_deg=CAMERA_V_FOV_DEG,
        )
        telemetry = self._telemetry.snapshot()

        for candidate in candidates:
            if not self._filter.should_publish(candidate.class_name, now):
                self.get_logger().debug(
                    f'[{candidate.class_name}] oczekiwanie na {DETECTION_MIN_COUNT} trafienia'
                )
                continue

            self._publish_confirmed(candidate, annotated, qr, camera, telemetry)

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

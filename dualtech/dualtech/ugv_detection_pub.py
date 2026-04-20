import cv2
import time
import threading
import queue
import os
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from dualtech_msgs.msg import Detection
from ultralytics import YOLO
from pyzbar.pyzbar import decode
from std_msgs.msg import String

TARGET_FPS = 20
FRAME_INTERVAL = 1.0 / TARGET_FPS
VIDEO_SOURCE = os.getenv("VIDEO_SOURCE", "csi").lower()
CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "1280"))
CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "720"))
CAMERA_FPS = int(os.getenv("CAMERA_FPS", "30"))

# GST_PIPELINE = (
#     "udpsrc port=5000 ! "
#     "application/x-rtp,encoding-name=H264,payload=96 ! "
#     "rtph264depay ! h264parse ! avdec_h264 ! "
#     "videoconvert ! appsink max-buffers=1 drop=true sync=false"
# )

GST_PIPELINE = (
    "libcamerasrc ! "
    "video/x-raw,format=NV12 ! "           # Najbardziej stabilny format dla RPi5
    "videoconvert ! "
    "videoscale ! "                        # Dodajemy skalowanie programowe/sprzętowe
    f"video/x-raw,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},format=BGR ! "
    "appsink drop=true max-buffers=1"
)
frame_queue = queue.Queue(maxsize=1)


class UGVDetectionPublisher(Node):
    def __init__(self):
        super().__init__('detection_publisher')
        self.publisher_ = self.create_publisher(Detection, 'detection', 10)
        self.model = YOLO("yolov8n_ground.pt")
        self.bridge = CvBridge()
        self.object_id_counter = 0
        self.get_logger().info(f"Video source mode: {VIDEO_SOURCE}")
        self.get_logger().info(f"GStreamer pipeline: {GST_PIPELINE}")
        self.get_logger().info("Detection publisher uruchomiony - pełny obraz z boxami")

    def publish_detections(self, frame, results):
        small = cv2.resize(frame, (480,360))
        decoded_objects = decode(small)

        # Sprawdzamy czy są jakiekolwiek detekcje
        if len(results[0].boxes) == 0:
            self.get_logger().debug("Brak detekcji")
            return

        qr=""
        
        if decoded_objects:
            for obj in decoded_objects:
                data = obj.data.decode('utf-8')

                qr=data
                self.get_logger().info(f"QR: {data}")
        else:
            self.get_logger().debug("Brak QR")

        # Metoda .plot() zwraca obraz (numpy array) z naniesionymi boxami i etykietami
        annotated_frame = results[0].plot()

        # Przygotowanie wiadomości
        msg = Detection()
        msg.object_id = self.object_id_counter

        # Opcjonalnie: tworzymy listę nazw wykrytych obiektów do pola object_type
        detected_classes = [self.model.names[int(box.cls[0])] for box in results[0].boxes]
        msg.object_type = ", ".join(set(detected_classes))  # Łączy unikalne nazwy w jeden string

        # Konwersja całego naniesionego obrazu na format ROS2
        msg.object_image = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        
        msg.qr_value=qr

        self.publisher_.publish(msg)
        self.get_logger().info(f"Opublikowano obraz z detekcjami: {msg.object_type}")
        self.object_id_counter += 1

    def run_yolo(self):
        last_time = time.time()
        while rclpy.ok():
            try:
                frame = frame_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            now = time.time()
            if now - last_time < FRAME_INTERVAL:
                continue
            last_time = now

            # Wykonanie detekcji
            results = self.model(frame, verbose=False, conf=0.5)
            self.publish_detections(frame, results)


def stream_reader():
    cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("[BŁĄD] Nie można otworzyć streamu GStreamer")
        return
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        if frame_queue.full():
            try:
                frame_queue.get_nowait()
            except queue.Empty:
                pass
        frame_queue.put(frame)


def main(args=None):
    rclpy.init(args=args)
    node = UGVDetectionPublisher()

    stream_thread = threading.Thread(target=stream_reader, daemon=True)
    stream_thread.start()

    yolo_thread = threading.Thread(target=node.run_yolo, daemon=True)
    yolo_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

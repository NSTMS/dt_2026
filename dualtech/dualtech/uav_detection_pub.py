import cv2
import time
import threading
import queue
import rclpy
import csv
from rclpy.node import Node
from cv_bridge import CvBridge
from dualtech_msgs.msg import Detection
from ultralytics import YOLO

# Importy dla telemetrii (standardowe dla MAVROS/DroneLink)
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

TARGET_FPS = 20
FRAME_INTERVAL = 1.0 / TARGET_FPS
LOG_INTERVAL = 1.0  # Wymóg rejestracji min. co 1s


class UAVDetectionPublisher(Node):
    def __init__(self):
        super().__init__('uav_detection_node')

        # --- YOLO & Obrazy ---
        self.publisher_ = self.create_publisher(Detection, '/yolo/detections', 10)
        self.model = YOLO("yolov8n_aerial.pt")
        self.bridge = CvBridge()

        # --- Telemetria (SpeedyBee via MAVROS/Micro-XRCE) ---
        # Subskrypcje danych z FC
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.baro_callback, 10)

        # Zmienne stanowe
        self.current_gps = {"lat": 0.0, "lon": 0.0, "alt_gps": 0.0, "time": 0.0}
        self.current_baro_alt = 0.0
        self.object_id_counter = 0

        # --- Rejestrator danych (CSV) ---
        self.log_file = open('flight_log.csv', 'a', newline='')
        self.csv_writer = csv.writer(self.log_file)
        # Nagłówki: GPS_Time, Lat, Lon, Alt_GPS, Alt_Baro
        self.csv_writer.writerow(['timestamp', 'lat', 'lon', 'alt_gps', 'alt_baro'])

        # Timer do rejestracji danych co 1s
        self.create_timer(LOG_INTERVAL, self.log_telemetry)

        self.get_logger().info("System UAV uruchomiony: Detekcja + Rejestracja danych")

    def gps_callback(self, msg):
        self.current_gps = {
            "lat": msg.latitude,
            "lon": msg.longitude,
            "alt_gps": msg.altitude,
            "time": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }

    def baro_callback(self, msg):
        self.current_baro_alt = msg.data

    def log_telemetry(self):
        """Zapisuje dane do CSV co 1 sekundę"""
        row = [
            self.current_gps["time"],
            self.current_gps["lat"],
            self.current_gps["lon"],
            self.current_gps["alt_gps"],
            self.current_baro_alt
        ]
        self.csv_writer.writerow(row)
        self.log_file.flush()  # Wymuszenie zapisu na dysk (RPi 5 jest szybkie, ale bezpieczniej tak)

        # Tutaj możesz dodać wysyłanie wysokości baro do organizatorów w czasie rzeczywistym
        # self.send_to_organizers(self.current_baro_alt)

    def publish_detections(self, frame, results):
        if len(results[0].boxes) == 0:
            return

        annotated_frame = results[0].plot()
        msg = Detection()
        msg.object_id = self.object_id_counter
        detected_classes = [self.model.names[int(box.cls[0])] for box in results[0].boxes]
        msg.object_type = f"{', '.join(set(detected_classes))} | altitute: {self.current_baro_alt:.2f}m"
        msg.object_image = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

        self.publisher_.publish(msg)
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

            results = self.model(frame, verbose=False, conf=0.5)
            self.publish_detections(frame, results)


def main(args=None):
    rclpy.init(args=args)
    node = UAVDetectionPublisher()

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

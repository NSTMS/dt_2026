import csv
import os
import threading
from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


@dataclass
class DroneTelemetry:
    latitude: float = 0.0
    longitude: float = 0.0
    altitude_gps: float = 0.0
    altitude_baro: float = 0.0
    timestamp: float = 0.0
    has_gps: bool = False
    has_altitude: bool = False
    gps_fix: NavSatFix | None = None


class MavrosTelemetry:
    """Zbiera dane z mavros potrzebne do geolokalizacji detekcji."""

    def __init__(self, node: Node):
        self._node = node
        self._lock = threading.Lock()
        self._state = DroneTelemetry()

        node.create_subscription(
            NavSatFix, '/mavros/global_position/global', self._on_gps, 10)
        node.create_subscription(
            Float64, '/mavros/global_position/rel_alt', self._on_baro, 10)
        node.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._on_pose, 10)

        log_path = os.getenv('FLIGHT_LOG_PATH', 'flight_log.csv')
        self._log_path = log_path
        write_header = not os.path.exists(log_path) or os.path.getsize(log_path) == 0
        self._log_file = open(log_path, 'a', newline='')
        self._csv_writer = csv.writer(self._log_file)
        if write_header:
            self._csv_writer.writerow(['timestamp', 'lat', 'lon', 'alt_gps', 'alt_baro'])
            self._log_file.flush()

        node.create_timer(1.0, self._log_telemetry)
        node.get_logger().info(f'Telemetria mavros, log lotu: {log_path}')

    def snapshot(self) -> DroneTelemetry:
        with self._lock:
            return DroneTelemetry(
                latitude=self._state.latitude,
                longitude=self._state.longitude,
                altitude_gps=self._state.altitude_gps,
                altitude_baro=self._state.altitude_baro,
                timestamp=self._state.timestamp,
                has_gps=self._state.has_gps,
                has_altitude=self._state.has_altitude,
                gps_fix=self._state.gps_fix,
            )

    def close(self) -> None:
        if self._log_file and not self._log_file.closed:
            self._log_file.close()

    def _on_gps(self, msg: NavSatFix) -> None:
        with self._lock:
            self._state.gps_fix = msg
            self._state.latitude = msg.latitude
            self._state.longitude = msg.longitude
            self._state.altitude_gps = msg.altitude
            self._state.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._state.has_gps = True

    def _on_baro(self, msg: Float64) -> None:
        with self._lock:
            self._state.altitude_baro = msg.data
            self._state.has_altitude = True

    def _on_pose(self, msg: PoseStamped) -> None:
        # Rezerwowane pod przyszłe użycie orientacji kamery / offsetu gimbala.
        pass

    def _log_telemetry(self) -> None:
        state = self.snapshot()
        self._csv_writer.writerow([
            state.timestamp,
            state.latitude,
            state.longitude,
            state.altitude_gps,
            state.altitude_baro,
        ])
        self._log_file.flush()

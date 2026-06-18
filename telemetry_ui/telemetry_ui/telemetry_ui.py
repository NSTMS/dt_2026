#!/usr/bin/env python3
"""
ROS2 + PyQt5 desktop dashboard for drone telemetry from mavros.

Shows live:
  - Battery voltage / current / percentage  (/mavros/battery)
  - Relative height                         (/mavros/global_position/rel_alt)
  - GPS localization (lat, lon, fix)        (/mavros/global_position/global)
  - Flight-controller status messages       (/mavros/statustext/recv)

The node spins from the Qt event loop via a QTimer, so all ROS callbacks and
widget updates run on the main thread (no cross-thread races).

Run on a ground-station laptop that shares the Pi's ROS_DOMAIN_ID:
    ros2 run telemetry_ui telemetry_ui
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Float64

try:
    from mavros_msgs.msg import StatusText
    HAVE_STATUSTEXT = True
except ImportError:  # mavros_msgs may be unavailable on the viewing machine
    StatusText = None
    HAVE_STATUSTEXT = False

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
)

# Severity codes per MAVLink MAV_SEVERITY (used by mavros StatusText).
SEVERITY_NAMES = {
    0: 'EMERGENCY',
    1: 'ALERT',
    2: 'CRITICAL',
    3: 'ERROR',
    4: 'WARNING',
    5: 'NOTICE',
    6: 'INFO',
    7: 'DEBUG',
}

# NavSatStatus.status values.
GPS_FIX_NAMES = {
    -1: 'NO FIX',
    0: 'FIX',
    1: 'SBAS FIX',
    2: 'GBAS FIX',
}

STALE_AFTER_S = 3.0
MAX_FC_LINES = 200


class TelemetryNode(Node):
    """Subscribes to mavros telemetry topics and caches the latest values."""

    def __init__(self):
        super().__init__('telemetry_ui')

        # Coloring thresholds (overridable via ROS params); raw values always shown.
        self.battery_full_v = float(
            self.declare_parameter('battery_full_v', 25.2).value)
        self.battery_empty_v = float(
            self.declare_parameter('battery_empty_v', 21.0).value)
        self.warn_pct = float(self.declare_parameter('warn_pct', 30.0).value)
        self.crit_pct = float(self.declare_parameter('crit_pct', 15.0).value)

        # Latest cached values + last-update wall-clock time per source.
        self.battery = None
        self.battery_t = 0.0
        self.rel_alt = None
        self.rel_alt_t = 0.0
        self.gps = None
        self.gps_t = 0.0

        # Pending FC status lines for the GUI to drain.
        self.pending_fc = []

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        self.create_subscription(
            BatteryState, '/mavros/battery', self._on_battery, sensor_qos)
        self.create_subscription(
            Float64, '/mavros/global_position/rel_alt', self._on_rel_alt, 10)
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self._on_gps,
            sensor_qos)

        if HAVE_STATUSTEXT:
            self.create_subscription(
                StatusText, '/mavros/statustext/recv', self._on_statustext, 10)
        else:
            self.get_logger().warn(
                'mavros_msgs not found; FC status messages disabled.')

        self.get_logger().info('Telemetry UI node started.')

    def _on_battery(self, msg: BatteryState) -> None:
        self.battery = msg
        self.battery_t = time.monotonic()

    def _on_rel_alt(self, msg: Float64) -> None:
        self.rel_alt = msg.data
        self.rel_alt_t = time.monotonic()

    def _on_gps(self, msg: NavSatFix) -> None:
        self.gps = msg
        self.gps_t = time.monotonic()

    def _on_statustext(self, msg) -> None:
        severity = SEVERITY_NAMES.get(int(msg.severity), str(msg.severity))
        stamp = time.strftime('%H:%M:%S')
        self.pending_fc.append(f'[{stamp}][{severity}] {msg.text}')


def _is_stale(t: float) -> bool:
    return t == 0.0 or (time.monotonic() - t) > STALE_AFTER_S


class TelemetryWindow(QMainWindow):
    """Main window rendering cached telemetry; drives rclpy from a QTimer."""

    def __init__(self, node: TelemetryNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('Drone Telemetry (mavros)')
        self.resize(440, 640)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(10)

        root.addWidget(self._build_battery_group())
        root.addWidget(self._build_altitude_group())
        root.addWidget(self._build_gps_group())
        root.addWidget(self._build_fc_group(), stretch=1)

        # Drive ROS spinning + UI refresh from the Qt loop (single thread).
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(100)  # 10 Hz

    # ---- group builders ----------------------------------------------------

    def _big_label(self) -> QLabel:
        lbl = QLabel('-- (no data)')
        f = QFont()
        f.setPointSize(28)
        f.setBold(True)
        lbl.setFont(f)
        lbl.setAlignment(Qt.AlignCenter)
        return lbl

    def _build_battery_group(self) -> QGroupBox:
        box = QGroupBox('Battery')
        layout = QVBoxLayout(box)
        self.battery_voltage = self._big_label()
        self.battery_detail = QLabel('current: --   charge: --')
        self.battery_detail.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.battery_voltage)
        layout.addWidget(self.battery_detail)
        self._battery_box = box
        return box

    def _build_altitude_group(self) -> QGroupBox:
        box = QGroupBox('Height')
        layout = QVBoxLayout(box)
        self.altitude_rel = self._big_label()
        self.altitude_gps = QLabel('GPS alt (MSL): --')
        self.altitude_gps.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.altitude_rel)
        layout.addWidget(self.altitude_gps)
        return box

    def _build_gps_group(self) -> QGroupBox:
        box = QGroupBox('GPS localization')
        layout = QVBoxLayout(box)
        self.gps_fix = QLabel('fix: -- (no data)')
        self.gps_fix.setAlignment(Qt.AlignCenter)
        coords = QHBoxLayout()
        self.gps_lat = QLabel('lat: --')
        self.gps_lon = QLabel('lon: --')
        self.gps_lat.setAlignment(Qt.AlignCenter)
        self.gps_lon.setAlignment(Qt.AlignCenter)
        coords.addWidget(self.gps_lat)
        coords.addWidget(self.gps_lon)
        layout.addWidget(self.gps_fix)
        layout.addLayout(coords)
        return box

    def _build_fc_group(self) -> QGroupBox:
        box = QGroupBox('Flight controller messages')
        layout = QVBoxLayout(box)
        self.fc_log = QPlainTextEdit()
        self.fc_log.setReadOnly(True)
        self.fc_log.setMaximumBlockCount(MAX_FC_LINES)
        mono = QFont('Monospace')
        mono.setStyleHint(QFont.TypeWriter)
        self.fc_log.setFont(mono)
        if not HAVE_STATUSTEXT:
            self.fc_log.setPlainText('(mavros_msgs unavailable on this machine)')
        layout.addWidget(self.fc_log)
        return box

    # ---- refresh -----------------------------------------------------------

    def _tick(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self._refresh_battery()
        self._refresh_altitude()
        self._refresh_gps()
        self._drain_fc()

    def _set_box_color(self, box: QGroupBox, color: str) -> None:
        box.setStyleSheet(
            f'QGroupBox {{ background-color: {color}; }}' if color else '')

    def _refresh_battery(self) -> None:
        node = self.node
        if node.battery is None or _is_stale(node.battery_t):
            self.battery_voltage.setText('-- (no data)')
            self.battery_detail.setText('current: --   charge: --')
            self._set_box_color(self._battery_box, '')
            return

        msg = node.battery
        voltage = msg.voltage
        self.battery_voltage.setText(f'{voltage:.1f} V')

        pct = msg.percentage * 100.0 if msg.percentage >= 0.0 else None
        current = msg.current
        current_txt = f'{current:.1f} A' if current == current else '--'  # NaN guard
        if pct is not None and pct >= 0.0:
            charge_txt = f'{pct:.0f} %'
        else:
            charge_txt = '--'
        self.battery_detail.setText(f'current: {current_txt}   charge: {charge_txt}')

        # Color from percentage if available, otherwise from voltage range.
        if pct is not None and pct >= 0.0:
            level = pct
            crit, warn = node.crit_pct, node.warn_pct
        else:
            span = max(node.battery_full_v - node.battery_empty_v, 1e-3)
            level = (voltage - node.battery_empty_v) / span * 100.0
            crit, warn = node.crit_pct, node.warn_pct

        if level <= crit:
            self._set_box_color(self._battery_box, '#e74c3c')
        elif level <= warn:
            self._set_box_color(self._battery_box, '#f1c40f')
        else:
            self._set_box_color(self._battery_box, '#2ecc71')

    def _refresh_altitude(self) -> None:
        node = self.node
        if node.rel_alt is None or _is_stale(node.rel_alt_t):
            self.altitude_rel.setText('-- (no data)')
        else:
            self.altitude_rel.setText(f'{node.rel_alt:.1f} m')

        if node.gps is None or _is_stale(node.gps_t):
            self.altitude_gps.setText('GPS alt (MSL): --')
        else:
            self.altitude_gps.setText(f'GPS alt (MSL): {node.gps.altitude:.1f} m')

    def _refresh_gps(self) -> None:
        node = self.node
        if node.gps is None or _is_stale(node.gps_t):
            self.gps_fix.setText('fix: -- (no data)')
            self.gps_lat.setText('lat: --')
            self.gps_lon.setText('lon: --')
            return

        msg = node.gps
        fix = GPS_FIX_NAMES.get(int(msg.status.status), str(msg.status.status))
        self.gps_fix.setText(f'fix: {fix}')
        self.gps_lat.setText(f'lat: {msg.latitude:.6f}')
        self.gps_lon.setText(f'lon: {msg.longitude:.6f}')

    def _drain_fc(self) -> None:
        if not self.node.pending_fc:
            return
        lines = self.node.pending_fc
        self.node.pending_fc = []
        for line in lines:
            self.fc_log.appendPlainText(line)

    def closeEvent(self, event):
        self._timer.stop()
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()

    app = QApplication(sys.argv)
    window = TelemetryWindow(node)
    window.show()

    try:
        exit_code = app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
ROS2 Node: rc_listener

Reads RC input from mavros (RadioMaster Pocket → FCU → mavros → /mavros/rc/in),
detects rising/falling edges on the release channel (default CH8), and publishes
servo commands on /servo/rc_command (std_msgs/Int8):
    1  → forward sweep (open)   — rising edge  (LOW→HIGH)
   -1  → reverse sweep (close)  — falling edge (HIGH→LOW)

On every RC update the current channel state is logged at DEBUG level. Enable it
with:
    ros2 run uav_detection rc_listener_node --ros-args --log-level rc_listener:=debug
"""

import os

import rclpy
from mavros_msgs.msg import RCIn
from rclpy.node import Node
from std_msgs.msg import Int8

# ── RC trigger (RadioMaster Pocket → mavros → CH8) ─────────────────────────────
RC_RELEASE_CHANNEL = int(os.getenv('RC_RELEASE_CHANNEL', '8'))   # kanał 1..18
RC_PWM_OFF = int(os.getenv('RC_PWM_OFF', '1000'))                  # wyłączony
RC_PWM_ON = int(os.getenv('RC_PWM_ON', '2000'))                    # włączony
RC_PWM_THRESHOLD = int(os.getenv('RC_PWM_THRESHOLD', '1500'))      # próg HIGH

# ── Servo command values published on /servo/rc_command ────────────────────────
RC_COMMAND_FORWARD = 1
RC_COMMAND_REVERSE = -1


class RcListenerNode(Node):
    """
    Watches the RC release channel and translates edges into servo commands:
      - rising edge (LOW→HIGH): publish RC_COMMAND_FORWARD (1).
      - falling edge (HIGH→LOW): publish RC_COMMAND_REVERSE (-1).
    The raw channel PWM and HIGH/LOW state are logged at DEBUG on every message.
    """

    def __init__(self):
        super().__init__('rc_listener')

        self.pub_command = self.create_publisher(Int8, '/servo/rc_command', 10)
        self.create_subscription(RCIn, '/mavros/rc/in', self._rc_callback, 10)

        self._rc_channel_was_high = False

        self.get_logger().info(
            f'RC listener: CH{RC_RELEASE_CHANNEL} '
            f'({RC_PWM_OFF}=OFF, {RC_PWM_ON}=ON, próg={RC_PWM_THRESHOLD}) '
            f'→ /servo/rc_command'
        )

    def _rc_callback(self, msg: RCIn):
        """
        RC channel edge triggers:
          - rising edge (LOW→HIGH, 1000→2000): publish forward command.
          - falling edge (HIGH→LOW, 2000→1000): publish reverse command.
        """
        ch_idx = RC_RELEASE_CHANNEL - 1
        if ch_idx < 0 or ch_idx >= len(msg.channels):
            return

        pwm = int(msg.channels[ch_idx])
        is_high = pwm >= RC_PWM_THRESHOLD

        self.get_logger().debug(
            f'RC CH{RC_RELEASE_CHANNEL}: pwm={pwm} µs, state={"HIGH" if is_high else "LOW"}'
        )

        if is_high and not self._rc_channel_was_high:
            self.get_logger().info(
                f'RC CH{RC_RELEASE_CHANNEL} HIGH ({pwm} µs) → forward command'
            )
            self._publish_command(RC_COMMAND_FORWARD)
        elif not is_high and self._rc_channel_was_high:
            self.get_logger().info(
                f'RC CH{RC_RELEASE_CHANNEL} LOW ({pwm} µs) → reverse command'
            )
            self._publish_command(RC_COMMAND_REVERSE)

        self._rc_channel_was_high = is_high

    def _publish_command(self, command: int):
        msg = Int8()
        msg.data = command
        self.pub_command.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RcListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

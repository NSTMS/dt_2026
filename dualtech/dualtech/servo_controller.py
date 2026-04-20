#!/usr/bin/env python3
"""
ROS2 Node: servo_controller
Controls a TowerPro SG90-HV servo on GPIO18 (Pin 12) using lgpio (RPi5 compatible).
A configurable sweep is triggered by pressing the configured keyboard key.

Wiring:
  - Servo control wire → GPIO18 (Physical Pin 12)
  - Servo power        → external 5V supply
  - Common ground      → shared with RPi

Dependencies:
  sudo apt install python3-lgpio python3-pynput
  pip install rclpy std_msgs
"""

import time
import threading

import lgpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from pynput import keyboard


# ─── USER CONSTANTS ────────────────────────────────────────────────────────────

TRIGGER_KEY     = keyboard.Key.space    # Key that triggers the sweep.
                                        # Examples:
                                        #   keyboard.Key.space            → Spacebar
                                        #   keyboard.Key.enter            → Enter
                                        #   keyboard.KeyCode.from_char('s') → 's'

SERVO_GPIO      = 17        # GPIO18 = Physical Pin 12

# ── Servo pulse width limits (microseconds) ────────────────────────────────────
#    These define the physical range of the servo.
#    Do not exceed the hardware limits of the SG90-HV (500–2500 µs).
SERVO_PW_MIN    = 600       # Absolute minimum  → full counter-clockwise
SERVO_PW_MAX    = 2400      # Absolute maximum  → full clockwise

# ── Home position ──────────────────────────────────────────────────────────────
#    Where the servo parks on startup and after every sweep.
#    Set to SERVO_PW_MAX for fully clockwise home.
SERVO_HOME_PW   = 2400      # Home = fully clockwise

# ── Sweep range ────────────────────────────────────────────────────────────────
#    Define where the sweep starts and ends (in µs, within SERVO_PW_MIN/MAX).
#    The sweep always goes: HOME → SWEEP_START → SWEEP_END → HOME.
SWEEP_START_PW  = 2400      # Sweep start position (fully clockwise)
SWEEP_END_PW    = 600       # Sweep end position   (fully counter-clockwise)

# ── Motion parameters ──────────────────────────────────────────────────────────
SWEEP_STEPS     = 100       # Steps per sweep pass — higher = smoother
SWEEP_DELAY     = 0.015     # Seconds between steps (~1.5 s per pass)

# ── lgpio PWM settings ─────────────────────────────────────────────────────────
PWM_FREQUENCY   = 50        # Hz — standard servo frequency
PWM_RANGE       = 20000     # µs period at 50 Hz (1 000 000 / 50)


# ─── HELPERS ───────────────────────────────────────────────────────────────────

def _pw_to_duty(pulse_width_us: int) -> float:
    """Convert pulse width in µs to a duty cycle value for lgpio (0–PWM_RANGE)."""
    return (pulse_width_us / PWM_RANGE) * PWM_RANGE  # lgpio uses absolute µs via tx_servo


# ─── NODE ──────────────────────────────────────────────────────────────────────

class ServoControllerNode(Node):
    """
    ROS2 node that:
      - Watches a keyboard key and triggers a configurable servo sweep on press.
      - Publishes sweep status on /servo/status  (std_msgs/String).
      - Publishes sweep active flag on /servo/active (std_msgs/Bool).
      - Subscribes to /servo/trigger (std_msgs/Bool) for remote triggering.

    Sweep profile:
      HOME → SWEEP_START → SWEEP_END → HOME
    All positions are defined as constants at the top of this file.
    """

    def __init__(self):
        super().__init__('servo_controller')

        # ── lgpio setup ──────────────────────────────────────────────────────
        # RPi5 uses gpiochip4; earlier models use gpiochip0.
        # lgpio auto-selects the correct chip based on the board revision.
        try:
            self._chip = lgpio.gpiochip_open(4)   # RPi5: chip 4
        except lgpio.error:
            self._chip = lgpio.gpiochip_open(0)   # fallback for older Pi models

        lgpio.gpio_claim_output(self._chip, SERVO_GPIO)
        self._set_servo(SERVO_HOME_PW)
        self.get_logger().info(
            f'Servo initialised on GPIO{SERVO_GPIO} — home position: {SERVO_HOME_PW} µs'
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_status = self.create_publisher(String, '/servo/status', 10)
        self.pub_active = self.create_publisher(Bool,   '/servo/active', 10)

        # ── Subscriber ───────────────────────────────────────────────────────
        self.create_subscription(Bool, '/servo/trigger', self._trigger_callback, 10)

        # ── Internal state ───────────────────────────────────────────────────
        self._sweep_lock    = threading.Lock()
        self._sweep_running = False

        # ── Keyboard listener ────────────────────────────────────────────────
        self._kb_listener = keyboard.Listener(on_press=self._on_key_press)
        self._kb_listener.start()
        self.get_logger().info(f'Keyboard listener started — trigger key: {TRIGGER_KEY}')
        self.get_logger().info(
            f'Sweep profile: HOME({SERVO_HOME_PW}µs) → '
            f'START({SWEEP_START_PW}µs) → END({SWEEP_END_PW}µs) → HOME({SERVO_HOME_PW}µs)'
        )
        self.get_logger().info('servo_controller node ready.')

    # ── Servo output ─────────────────────────────────────────────────────────

    def _set_servo(self, pulse_width_us: int):
        """Send a PWM pulse to the servo. Clamps to hardware limits."""
        pw = max(SERVO_PW_MIN, min(SERVO_PW_MAX, pulse_width_us))
        lgpio.tx_servo(self._chip, SERVO_GPIO, pw, PWM_FREQUENCY)

    def _servo_off(self):
        """Stop sending PWM pulses (servo relaxes, saves power)."""
        lgpio.tx_servo(self._chip, SERVO_GPIO, 0, PWM_FREQUENCY)

    # ── Sweep logic ──────────────────────────────────────────────────────────

    def _do_sweep(self):
        """
        Execute sweep profile:
          HOME → SWEEP_START → SWEEP_END → HOME
        """
        with self._sweep_lock:
            if self._sweep_running:
                self.get_logger().warn('Sweep already in progress — ignoring request.')
                return
            self._sweep_running = True

        self._publish_active(True)
        self._publish_status('Sweep started')
        self.get_logger().info('Starting sweep')

        try:
            # Move to sweep start if not already there
            if SERVO_HOME_PW != SWEEP_START_PW:
                self.get_logger().info(f'Moving to sweep start ({SWEEP_START_PW} µs)')
                self._sweep_range(SERVO_HOME_PW, SWEEP_START_PW)

            # Execute the main sweep
            self.get_logger().info(
                f'Sweeping: {SWEEP_START_PW} µs → {SWEEP_END_PW} µs'
            )
            self._sweep_range(SWEEP_START_PW, SWEEP_END_PW)

            # Return to home
            self.get_logger().info(f'Returning home ({SERVO_HOME_PW} µs)')
            self._sweep_range(SWEEP_END_PW, SERVO_HOME_PW)

            self._publish_status('Sweep complete')
            self.get_logger().info('Sweep complete — servo at home position')

        except Exception as e:
            self.get_logger().error(f'Sweep error: {e}')
            self._publish_status(f'Sweep error: {e}')

        finally:
            with self._sweep_lock:
                self._sweep_running = False
            self._publish_active(False)

    def _sweep_range(self, pw_from: int, pw_to: int):
        """Smoothly move servo from pw_from to pw_to (µs) over SWEEP_STEPS steps."""
        for step in range(SWEEP_STEPS + 1):
            pw = int(pw_from + (pw_to - pw_from) * step / SWEEP_STEPS)
            self._set_servo(pw)
            time.sleep(SWEEP_DELAY)

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_key_press(self, key):
        """Keyboard press handler (runs in pynput thread)."""
        if key == TRIGGER_KEY:
            self.get_logger().info(f'Trigger key pressed: {TRIGGER_KEY}')
            threading.Thread(target=self._do_sweep, daemon=True).start()

    def _trigger_callback(self, msg: Bool):
        """Remote trigger via /servo/trigger topic."""
        if msg.data:
            self.get_logger().info('Remote trigger received on /servo/trigger')
            threading.Thread(target=self._do_sweep, daemon=True).start()

    # ── Publisher helpers ─────────────────────────────────────────────────────

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _publish_active(self, active: bool):
        msg = Bool()
        msg.data = active
        self.pub_active.publish(msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info('Shutting down — returning to home and releasing GPIO')
        self._set_servo(SERVO_HOME_PW)
        time.sleep(0.5)
        self._servo_off()
        self._kb_listener.stop()
        lgpio.gpiochip_close(self._chip)
        super().destroy_node()


# ─── ENTRY POINT ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

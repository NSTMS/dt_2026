#!/usr/bin/env python3
"""
ROS2 Node: servo_controller
Controls a TowerPro SG90-HV servo on GPIO18 (Pin 12) using lgpio (RPi5 compatible).
Keyboard trigger reads directly from stdin — works over SSH with no display needed.

Wiring:
  - Servo control wire → GPIO18 (Physical Pin 12)
  - Servo power        → external 5V supply
  - Common ground      → shared with RPi

Dependencies:
  sudo apt install python3-lgpio
  pip install readchar
"""

import sys
import tty
import termios
import threading
import time

import lgpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


# ─── USER CONSTANTS ────────────────────────────────────────────────────────────

TRIGGER_KEY     = ' '       # Keyboard key that triggers the sweep (single char).
                            # Examples:
                            #   ' '  → Spacebar
                            #   '\n' → Enter
                            #   's'  → s key

REVERSE_KEY     = '\n'
SERVO_GPIO      = 17       # GPIO18 = Physical Pin 12

# ── Servo pulse width limits (microseconds) ────────────────────────────────────
#    Do not exceed the hardware limits of the SG90-HV (500–2500 µs).
SERVO_PW_MIN    = 500       # Absolute minimum  → full counter-clockwise
SERVO_PW_MAX    = 2500      # Absolute maximum  → full clockwise

# ── Home position ──────────────────────────────────────────────────────────────
#    Where the servo parks on startup and returns after every sweep.
SERVO_HOME_PW   = 1500      # Home = fully clockwise

# ── Sweep range ────────────────────────────────────────────────────────────────
#    Sweep profile: HOME → SWEEP_START → SWEEP_END → HOME
SWEEP_START_PW  = 1500      # Sweep start position (fully clockwise)
SWEEP_END_PW    = 1800    # Sweep end position   (fully counter-clockwise)

# ── Motion parameters ──────────────────────────────────────────────────────────
SWEEP_STEPS     = 43      # Steps per pass — higher = smoother
SWEEP_DELAY     = 0.015     # Seconds between steps (~1.5 s per pass)

# ── lgpio PWM settings ─────────────────────────────────────────────────────────
PWM_FREQUENCY   = 50        # Hz — standard servo frequency


# ─── STDIN KEYBOARD READER ─────────────────────────────────────────────────────

class StdinKeyReader:
    """
    Reads single keypresses directly from stdin in raw mode.
    Works over SSH without any display server.
    Calls `callback(char)` for every key pressed.
    """

    def __init__(self, callback):
        self._callback = callback
        self._running  = False
        self._thread   = threading.Thread(target=self._run, daemon=True)
        self._old_settings = None

    def start(self):
        self._running = True
        self._thread.start()

    def stop(self):
        self._running = False
        # Restore terminal if we changed it
        if self._old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            except Exception:
                pass

    def _run(self):
        # Only switch to raw mode if stdin is a real TTY (i.e. interactive SSH)
        if not sys.stdin.isatty():
            return

        fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self._running:
                ch = sys.stdin.read(1)
                if ch:
                    self._callback(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, self._old_settings)


# ─── NODE ──────────────────────────────────────────────────────────────────────

class ServoControllerNode(Node):
    """
    ROS2 node that:
      - Watches stdin for a keypress and triggers a servo sweep.
      - Publishes sweep status on  /servo/status (std_msgs/String).
      - Publishes sweep active flag on /servo/active (std_msgs/Bool).
      - Subscribes to /servo/trigger (std_msgs/Bool) for remote triggering.

    Sweep profile:  HOME → SWEEP_START → SWEEP_END → HOME
    """

    def __init__(self):
        super().__init__('servo_controller')

        # ── lgpio setup ──────────────────────────────────────────────────────
        try:
            self._chip = lgpio.gpiochip_open(4)   # RPi5: gpiochip4
        except lgpio.error:
            self._chip = lgpio.gpiochip_open(0)   # RPi4 and older: gpiochip0

        lgpio.gpio_claim_output(self._chip, SERVO_GPIO)
        self._set_servo(SERVO_HOME_PW)
        time.sleep(0.5)       # Allow servo to physically reach home before cutting signal
        self._servo_off()     # Cut PWM — servo holds position passively, stops jittering
        self.get_logger().info(
            f'Servo initialised on GPIO{SERVO_GPIO} — home: {SERVO_HOME_PW} µs'
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_status = self.create_publisher(String, '/servo/status', 10)
        self.pub_active = self.create_publisher(Bool,   '/servo/active', 10)

        # ── Subscriber ───────────────────────────────────────────────────────
        self.create_subscription(Bool, '/servo/trigger', self._trigger_callback, 10)

        # ── Internal state ───────────────────────────────────────────────────
        self._sweep_lock    = threading.Lock()
        self._sweep_running = False

        # ── Stdin keyboard listener ──────────────────────────────────────────
        self._key_reader = StdinKeyReader(callback=self._on_key_press)
        self._key_reader.start()

        trigger_display = 'SPACE' if TRIGGER_KEY == ' ' else repr(TRIGGER_KEY)
        self.get_logger().info(f'Keyboard trigger: [{trigger_display}]  |  Ctrl+C to exit')
        self.get_logger().info(
            f'Sweep: HOME({SERVO_HOME_PW}µs) → '
            f'START({SWEEP_START_PW}µs) → END({SWEEP_END_PW}µs) → HOME({SERVO_HOME_PW}µs)'
        )

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
        """Execute sweep: HOME → SWEEP_START → SWEEP_END → HOME."""
        with self._sweep_lock:
            if self._sweep_running:
                self.get_logger().warn('Sweep already in progress — ignoring.')
                return
            self._sweep_running = True

        self._publish_active(True)
        self._publish_status('Sweep started')
        self.get_logger().info('Starting sweep')

        try:
            if SERVO_HOME_PW != SWEEP_START_PW:
                self.get_logger().info(f'Moving to sweep start ({SWEEP_START_PW} µs)')
                self._sweep_range(SERVO_HOME_PW, SWEEP_START_PW)

            self.get_logger().info(f'Sweeping {SWEEP_START_PW} µs → {SWEEP_END_PW} µs')
            self._sweep_range(SWEEP_START_PW, SWEEP_END_PW)

            self.get_logger().info(f'Returning home ({SERVO_HOME_PW} µs)')
            self._sweep_range(SWEEP_END_PW, SERVO_HOME_PW)

            time.sleep(0.3)       # Let servo settle at home position
            self._servo_off()     # Cut PWM — stops jitter while idle
            self._publish_status('Sweep complete')
            self.get_logger().info('Sweep complete — servo at home, PWM off')

        except Exception as e:
            self.get_logger().error(f'Sweep error: {e}')
            self._publish_status(f'Sweep error: {e}')

        finally:
            with self._sweep_lock:
                self._sweep_running = False

    def _do_sweep_reverse(self):
        """Execute sweep: HOME → SWEEP_START → SWEEP_END → HOME."""
        with self._sweep_lock:
            if self._sweep_running:
                self.get_logger().warn('Sweep already in progress — ignoring.')
                return
            self._sweep_running = True

        self._publish_active(True)
        self._publish_status('Sweep started')
        self.get_logger().info('Starting sweep')

        try:
            if SERVO_HOME_PW != SWEEP_START_PW:
                self.get_logger().info(f'Moving to sweep start ({SWEEP_START_PW} µs)')
                self._sweep_range(SERVO_HOME_PW, SWEEP_START_PW)

            self.get_logger().info(f'Sweeping {SWEEP_START_PW} µs → {SWEEP_END_PW} µs')
            self._sweep_range(SWEEP_START_PW, 1200)

            self.get_logger().info(f'Returning home ({SERVO_HOME_PW} µs)')
            self._sweep_range(1200, SERVO_HOME_PW)

            time.sleep(0.3)       # Let servo settle at home position
            self._servo_off()     # Cut PWM — stops jitter while idle
            self._publish_status('Sweep complete')
            self.get_logger().info('Sweep complete — servo at home, PWM off')

        except Exception as e:
            self.get_logger().error(f'Sweep error: {e}')
            self._publish_status(f'Sweep error: {e}')

        finally:
            with self._sweep_lock:
                self._sweep_running = False
            self._publish_active(False)

            self._publish_active(False)

    def _sweep_range(self, pw_from: int, pw_to: int):
        """Smoothly interpolate servo from pw_from to pw_to."""
        for step in range(SWEEP_STEPS + 1):
            pw = int(pw_from + (pw_to - pw_from) * step / SWEEP_STEPS)
            self._set_servo(pw)
            time.sleep(SWEEP_DELAY)

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_key_press(self, char: str):
        """Stdin keypress handler (runs in reader thread)."""
        if char == '\x03':          # Ctrl+C — propagate graceful shutdown
            raise KeyboardInterrupt
        if char == TRIGGER_KEY:
            self.get_logger().info('Trigger key pressed')
            threading.Thread(target=self._do_sweep, daemon=True).start()
        if char == REVERSE_KEY:
            self.get_logger().info('Reverse key pressed')
            threading.Thread(target=self._do_sweep_reverse, daemon=True).start()

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
        self._key_reader.stop()
        self._set_servo(SERVO_HOME_PW)
        time.sleep(0.5)
        self._servo_off()
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

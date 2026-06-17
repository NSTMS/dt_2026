#!/usr/bin/env python3
"""
ROS2 Node: servo_controller
Controls a TowerPro SG90-HV 360° continuous-rotation servo on GPIO17 (Pin 11)
using lgpio (RPi5 compatible).
Keyboard trigger reads directly from stdin — works over SSH with no display needed.

Continuous-rotation servo model:
  Unlike a positional servo, a 360° continuous-rotation servo has NO fixed angle.
  The pulse width sets the rotation DIRECTION and SPEED:
    pw == NEUTRAL  → stopped (servo holds still)
    pw  > NEUTRAL  → rotate one way   (speed ∝ pw − NEUTRAL)
    pw  < NEUTRAL  → rotate other way (speed ∝ NEUTRAL − pw)
  A "sweep" is therefore: spin up to a rotation speed, then spin back down to a
  stop — the time spent rotating determines how far the output shaft travels.

Wiring:
  - Servo control wire → GPIO17 (Physical Pin 11)
  - Servo power        → external 5V supply
  - Common ground      → shared with RPi

Dependencies:
  sudo apt install python3-lgpio
  pip install readchar
"""

import os
import sys
import tty
import termios
import threading
import time

import lgpio
import rclpy
from mavros_msgs.msg import RCIn
from rclpy.node import Node
from std_msgs.msg import Bool, String


# ─── USER CONSTANTS ────────────────────────────────────────────────────────────

TRIGGER_KEY     = ' '       # Key that starts a FORWARD sweep (single char).
                            # Examples:
                            #   ' '  → Spacebar
                            #   '\n' → Enter
                            #   's'  → s key

REVERSE_KEY     = 'c'      # Key that starts a REVERSE sweep — only accepted
                            # when the servo is currently 'opened'.

SERVO_GPIO      = 17        # GPIO17 = Physical Pin 11

# ── Logical servo positions ────────────────────────────────────────────────────
#    Track the mechanism state so sweeps are only triggered when valid:
#      forward sweep OPENS  (closed → opened)
#      reverse sweep CLOSES (opened → closed)
POSITION_OPENED = 'opened'
POSITION_CLOSED = 'closed'

# ── Servo pulse width limits (microseconds) ────────────────────────────────────
#    Do not exceed the hardware limits of the SG90-HV (500–2500 µs).
SERVO_PW_MIN    = 500       # Absolute minimum pulse width
SERVO_PW_MAX    = 2500      # Absolute maximum pulse width

# ── Neutral / stop ─────────────────────────────────────────────────────────────
#    Pulse width at which the continuous-rotation servo stays still.
SERVO_NEUTRAL_PW = 1500

# ── Sweep speed / direction ────────────────────────────────────────────────────
#    Pulse width applied during a forward sweep. The offset from neutral sets the
#    rotation speed; the sign of the offset sets the direction.
#    The reverse sweep mirrors this around neutral automatically, so reverse spins
#    the opposite way at the same speed.
#       forward speed pw = SWEEP_SPEED_PW                       (e.g. 1800)
#       reverse speed pw = 2*SERVO_NEUTRAL_PW − SWEEP_SPEED_PW  (e.g. 1200)
SWEEP_SPEED_PW   = 1800

# ── Motion profile ─────────────────────────────────────────────────────────────
#    Each sweep accelerates from a stop up to SWEEP_SPEED_PW, then decelerates
#    back to a stop. RAMP_STEPS × RAMP_DELAY ≈ duration of one accel/decel ramp.
#    Higher RAMP_STEPS = smoother; the total rotated arc is set by the ramp time.
SWEEP_RAMP_STEPS = 43       # Steps per ramp — higher = smoother
SWEEP_RAMP_DELAY = 0.015    # Seconds between steps (~0.65 s per ramp)

# ── lgpio PWM settings ─────────────────────────────────────────────────────────
PWM_FREQUENCY    = 50       # Hz — standard servo frequency

# ── RC trigger (RadioMaster Pocket → mavros → CH8) ─────────────────────────────
RC_RELEASE_CHANNEL = int(os.getenv('RC_RELEASE_CHANNEL', '8'))   # kanał 1..18
RC_PWM_OFF = int(os.getenv('RC_PWM_OFF', '1000'))                  # wyłączony
RC_PWM_ON = int(os.getenv('RC_PWM_ON', '2000'))                    # włączony
RC_PWM_THRESHOLD = int(os.getenv('RC_PWM_THRESHOLD', '1500'))      # próg HIGH


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
    ROS2 node that drives a 360° continuous-rotation servo. It tracks a logical
    position ('opened'/'closed') so sweeps are only triggered when valid:
      - FORWARD sweep = OPEN  (only when closed) on TRIGGER_KEY / /servo/trigger /
        RC CH8 rising edge.
      - REVERSE sweep = CLOSE (only when opened) on REVERSE_KEY / RC CH8 falling
        edge.
      - Publishes sweep status on  /servo/status (std_msgs/String).
      - Publishes sweep active flag on /servo/active (std_msgs/Bool).

    Sweep profile:  STOP → SPEED → STOP   (ramp up, ramp down)
    """

    def __init__(self):
        super().__init__('servo_controller')

        # ── Parameters ───────────────────────────────────────────────────────
        #    Current servo position = logical state 'opened' or 'closed'. It gates
        #    which sweep is allowed: a forward sweep OPENS (only when closed) and a
        #    reverse sweep CLOSES (only when opened). Set the startup state here so
        #    it matches the servo's real position. Configurable via launch.
        self.declare_parameter('servo_position', POSITION_CLOSED)
        position = str(self.get_parameter('servo_position').value).lower()
        if position not in (POSITION_OPENED, POSITION_CLOSED):
            self.get_logger().warn(
                f"Invalid servo_position '{position}' — defaulting to '{POSITION_CLOSED}'"
            )
            position = POSITION_CLOSED
        self._position = position
        self._neutral_pw = SERVO_NEUTRAL_PW

        # ── lgpio setup ──────────────────────────────────────────────────────
        try:
            self._chip = lgpio.gpiochip_open(4)   # RPi5: gpiochip4
        except lgpio.error:
            self._chip = lgpio.gpiochip_open(0)   # RPi4 and older: gpiochip0

        lgpio.gpio_claim_output(self._chip, SERVO_GPIO)
        self._set_servo(self._neutral_pw)
        time.sleep(0.5)       # Allow servo to settle at a stop before cutting signal
        self._servo_off()     # Cut PWM — continuous servo stays still, stops jittering
        self.get_logger().info(
            f'Servo initialised on GPIO{SERVO_GPIO} — neutral/stop: {self._neutral_pw} µs'
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_status = self.create_publisher(String, '/servo/status', 10)
        self.pub_active = self.create_publisher(Bool,   '/servo/active', 10)

        # ── Subscriber ───────────────────────────────────────────────────────
        self.create_subscription(Bool, '/servo/trigger', self._trigger_callback, 10)
        self.create_subscription(RCIn, '/mavros/rc/in', self._rc_callback, 10)

        # ── Internal state ───────────────────────────────────────────────────
        self._sweep_lock    = threading.Lock()
        self._sweep_running = False
        self._rc_channel_was_high = False

        # ── Stdin keyboard listener ──────────────────────────────────────────
        self._key_reader = StdinKeyReader(callback=self._on_key_press)
        self._key_reader.start()

        trigger_display = 'SPACE' if TRIGGER_KEY == ' ' else repr(TRIGGER_KEY)
        reverse_display = 'ENTER' if REVERSE_KEY == '\n' else repr(REVERSE_KEY)
        self.get_logger().info(
            f'Keys: forward=[{trigger_display}]  reverse=[{reverse_display}]  |  Ctrl+C to exit'
        )
        self.get_logger().info(
            f'Forward sweep (OPEN):  STOP({self._neutral_pw}µs) → SPEED({SWEEP_SPEED_PW}µs) → STOP'
        )
        self.get_logger().info(
            f'Reverse sweep (CLOSE): STOP({self._neutral_pw}µs) → SPEED({self._reverse_speed_pw()}µs) → STOP'
        )
        self.get_logger().info(f'Servo position at startup: {self._position.upper()}')
        self.get_logger().info(
            f'RC trigger: CH{RC_RELEASE_CHANNEL} '
            f'({RC_PWM_OFF}=OFF, {RC_PWM_ON}=ON, próg={RC_PWM_THRESHOLD})'
        )

    # ── Servo output ─────────────────────────────────────────────────────────

    def _set_servo(self, pulse_width_us: int):
        """Send a PWM pulse to the servo. Clamps to hardware limits."""
        pw = max(SERVO_PW_MIN, min(SERVO_PW_MAX, pulse_width_us))
        lgpio.tx_servo(self._chip, SERVO_GPIO, pw, PWM_FREQUENCY)

    def _servo_off(self):
        """Stop sending PWM pulses (servo relaxes, saves power)."""
        lgpio.tx_servo(self._chip, SERVO_GPIO, 0, PWM_FREQUENCY)

    def _reverse_speed_pw(self) -> int:
        """Pulse width that spins the servo the opposite way at the same speed."""
        return 2 * self._neutral_pw - SWEEP_SPEED_PW

    # ── Sweep logic ──────────────────────────────────────────────────────────

    def _run_sweep(self, speed_pw: int, label: str, new_position: str):
        """
        Execute one sweep on the continuous-rotation servo:
        ramp from a stop up to `speed_pw`, then ramp back down to a stop.
        The direction is encoded in `speed_pw` (offset/sign relative to neutral).
        On success the logical position is updated to `new_position`.
        """
        with self._sweep_lock:
            if self._sweep_running:
                self.get_logger().warn('Sweep already in progress — ignoring.')
                return
            self._sweep_running = True

        self._publish_active(True)
        self._publish_status(f'{label} sweep started')
        self.get_logger().info(f'Starting {label} sweep')

        completed = False
        try:
            self.get_logger().info(f'Accelerating to {speed_pw} µs')
            self._ramp(self._neutral_pw, speed_pw)

            self.get_logger().info(f'Decelerating to stop ({self._neutral_pw} µs)')
            self._ramp(speed_pw, self._neutral_pw)

            time.sleep(0.3)       # Let servo settle at a stop
            self._servo_off()     # Cut PWM — stops jitter while idle
            completed = True
            self._publish_status(f'{label} sweep complete')
            self.get_logger().info(f'{label} sweep complete — servo stopped, PWM off')

        except Exception as e:
            self.get_logger().error(f'Sweep error: {e}')
            self._publish_status(f'Sweep error: {e}')

        finally:
            with self._sweep_lock:
                self._sweep_running = False
                if completed:
                    # Advance the logical position so the opposite sweep is enabled.
                    self._position = new_position
            if completed:
                self.get_logger().info(f'Servo position: {new_position.upper()}')
            self._publish_active(False)

    def _do_forward_sweep(self):
        """Forward sweep = OPEN — only valid when the servo is currently closed."""
        with self._sweep_lock:
            allowed = (self._position == POSITION_CLOSED) and not self._sweep_running
        if not allowed:
            self.get_logger().warn(
                'Forward (open) ignored — servo is not closed.'
            )
            self._publish_status('Forward not allowed (already opened)')
            return
        self._run_sweep(SWEEP_SPEED_PW, 'forward', new_position=POSITION_OPENED)

    def _do_reverse_sweep(self):
        """Reverse sweep = CLOSE — only valid when the servo is currently opened."""
        with self._sweep_lock:
            allowed = (self._position == POSITION_OPENED) and not self._sweep_running
        if not allowed:
            self.get_logger().warn(
                'Reverse (close) ignored — servo is not opened.'
            )
            self._publish_status('Reverse not allowed (already closed)')
            return
        self._run_sweep(self._reverse_speed_pw(), 'reverse', new_position=POSITION_CLOSED)

    def _ramp(self, pw_from: int, pw_to: int):
        """Smoothly interpolate the servo drive signal from pw_from to pw_to."""
        for step in range(SWEEP_RAMP_STEPS + 1):
            pw = int(pw_from + (pw_to - pw_from) * step / SWEEP_RAMP_STEPS)
            self._set_servo(pw)
            time.sleep(SWEEP_RAMP_DELAY)

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_key_press(self, char: str):
        """Stdin keypress handler (runs in reader thread)."""
        if char == '\x03':          # Ctrl+C — propagate graceful shutdown
            raise KeyboardInterrupt
        if char == TRIGGER_KEY:
            self.get_logger().info('Forward key pressed')
            threading.Thread(target=self._do_forward_sweep, daemon=True).start()
        elif char == REVERSE_KEY:
            self.get_logger().info('Reverse key pressed')
            threading.Thread(target=self._do_reverse_sweep, daemon=True).start()

    def _trigger_callback(self, msg: Bool):
        """Remote trigger via /servo/trigger topic."""
        if msg.data:
            self.get_logger().info('Remote trigger received on /servo/trigger')
            threading.Thread(target=self._do_forward_sweep, daemon=True).start()

    def _rc_callback(self, msg: RCIn):
        """
        RC CH8 edge triggers:
          - rising edge (LOW→HIGH, 1000→2000): forward sweep.
          - falling edge (HIGH→LOW, 2000→1000): reverse sweep (if armed).
        """
        ch_idx = RC_RELEASE_CHANNEL - 1
        if ch_idx < 0 or ch_idx >= len(msg.channels):
            return

        pwm = int(msg.channels[ch_idx])
        is_high = pwm >= RC_PWM_THRESHOLD

        if is_high and not self._rc_channel_was_high:
            self.get_logger().info(
                f'RC CH{RC_RELEASE_CHANNEL} HIGH ({pwm} µs) → servo release'
            )
            threading.Thread(target=self._do_forward_sweep, daemon=True).start()
        elif not is_high and self._rc_channel_was_high:
            self.get_logger().info(
                f'RC CH{RC_RELEASE_CHANNEL} LOW ({pwm} µs) → reverse sweep'
            )
            threading.Thread(target=self._do_reverse_sweep, daemon=True).start()

        self._rc_channel_was_high = is_high

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
        self.get_logger().info('Shutting down — stopping servo and releasing GPIO')
        self._key_reader.stop()
        self._set_servo(self._neutral_pw)
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

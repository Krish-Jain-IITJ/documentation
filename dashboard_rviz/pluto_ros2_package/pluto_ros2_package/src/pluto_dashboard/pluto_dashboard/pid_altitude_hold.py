"""
PID Altitude Hold Node for Pluto Drone
=======================================
Subscribes to:
  /pluto/altitude   (sensor_msgs/Range)   — barometer
  /pluto/imu        (sensor_msgs/Imu)     — orientation + rates
  /pid/target_alt   (std_msgs/Float32)    — desired altitude (m)
  /pid/enable       (std_msgs/Bool)       — enable/disable hold
  /pid/gains        (std_msgs/String)     — JSON gains update

Publishes to:
  /drone_command    (custom_msgs/PlutoMsg) — overrides RC throttle
  /pid/state        (std_msgs/String)     — JSON state for dashboard

RC range: 1000–2000, neutral 1500.
Throttle hover baseline ≈ 1500. PID output is added on top.
"""

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float32, String

from custom_msgs.msg import PlutoMsg

NEUTRAL   = 1500
THR_MIN   = 1100
THR_MAX   = 1900
THR_HOVER = 1500   # baseline throttle to hover — tune per drone


class PIDController:
    """Simple PID with anti-windup and output clamping."""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                 out_min=-400.0, out_max=400.0, windup_limit=200.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.windup_limit = windup_limit

        self._integral  = 0.0
        self._prev_err  = 0.0
        self._prev_time = None

    def reset(self):
        self._integral  = 0.0
        self._prev_err  = 0.0
        self._prev_time = None

    def compute(self, setpoint: float, measured: float) -> float:
        now = time.monotonic()
        if self._prev_time is None:
            self._prev_time = now
            self._prev_err  = setpoint - measured
            return 0.0

        dt = now - self._prev_time
        if dt <= 0.0:
            return 0.0
        self._prev_time = now

        error = setpoint - measured

        # Proportional
        p = self.kp * error

        # Integral with anti-windup clamp
        self._integral += error * dt
        self._integral = max(-self.windup_limit,
                             min(self.windup_limit, self._integral))
        i = self.ki * self._integral

        # Derivative (on measurement to avoid derivative kick)
        d = self.kd * (error - self._prev_err) / dt
        self._prev_err = error

        output = p + i + d
        return max(self.out_min, min(self.out_max, output))


class AltitudeHoldNode(Node):

    def __init__(self):
        super().__init__('pluto_altitude_hold')

        # ── PID gains (tuned for Pluto, adjust via dashboard) ──────
        self.pid = PIDController(
            kp=120.0,   # proportional — main hover correction
            ki=15.0,    # integral     — removes steady-state drift
            kd=80.0,    # derivative   — damps oscillation
            out_min=-350.0,
            out_max=350.0,
            windup_limit=150.0,
        )

        # ── State ──────────────────────────────────────────────────
        self.enabled       = False
        self.target_alt    = 1.0        # metres above takeoff point
        self.current_alt   = 0.0
        self.takeoff_alt   = None       # set when hold is first enabled
        self.current_roll  = 0.0        # deg — for state reporting
        self.current_pitch = 0.0
        self._lock         = threading.Lock()

        # ── ROS publishers ─────────────────────────────────────────
        self.cmd_pub   = self.create_publisher(PlutoMsg, '/drone_command', 10)
        self.state_pub = self.create_publisher(String,   '/pid/state',     10)

        # ── ROS subscribers ────────────────────────────────────────
        self.create_subscription(Range,   '/pluto/altitude', self._alt_cb,    10)
        self.create_subscription(Imu,     '/pluto/imu',      self._imu_cb,    10)
        self.create_subscription(Bool,    '/pid/enable',     self._enable_cb,  10)
        self.create_subscription(Float32, '/pid/target_alt', self._target_cb,  10)
        self.create_subscription(String,  '/pid/gains',      self._gains_cb,   10)

        # ── Control loop timer — 20 Hz ─────────────────────────────
        self.create_timer(0.05, self._control_loop)

        # ── State publish timer — 10 Hz ────────────────────────────
        self.create_timer(0.1, self._publish_state)

        self.get_logger().info('AltitudeHold node ready. '
                               'Send True to /pid/enable to activate.')

    # ── Sensor callbacks ───────────────────────────────────────────

    def _alt_cb(self, msg: Range):
        with self._lock:
            self.current_alt = float(msg.range)

    def _imu_cb(self, msg: Imu):
        """Convert quaternion → roll/pitch degrees for state reporting."""
        q = msg.orientation
        # roll (x-axis)
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.degrees(math.atan2(sinr, cosr))
        # pitch (y-axis)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.degrees(math.asin(sinp))
        with self._lock:
            self.current_roll  = roll
            self.current_pitch = pitch

    # ── Command callbacks ──────────────────────────────────────────

    def _enable_cb(self, msg: Bool):
        with self._lock:
            if msg.data and not self.enabled:
                # Latch current altitude as reference zero on enable
                self.takeoff_alt = self.current_alt
                self.pid.reset()
                self.enabled = True
                self.get_logger().info(
                    f'Altitude hold ENABLED. '
                    f'Takeoff ref={self.takeoff_alt:.2f}m, '
                    f'target={self.target_alt:.2f}m above ref.'
                )
            elif not msg.data and self.enabled:
                self.enabled = False
                self.pid.reset()
                self.get_logger().info('Altitude hold DISABLED.')

    def _target_cb(self, msg: Float32):
        with self._lock:
            self.target_alt = float(msg.data)
            self.get_logger().info(f'Target altitude set to {self.target_alt:.2f}m')

    def _gains_cb(self, msg: String):
        """Accept JSON gains: {"kp":120,"ki":15,"kd":80}"""
        try:
            g = json.loads(msg.data)
            with self._lock:
                if 'kp' in g: self.pid.kp = float(g['kp'])
                if 'ki' in g: self.pid.ki = float(g['ki'])
                if 'kd' in g: self.pid.kd = float(g['kd'])
                self.pid.reset()
            self.get_logger().info(
                f'PID gains updated: kp={self.pid.kp} '
                f'ki={self.pid.ki} kd={self.pid.kd}'
            )
        except Exception as e:
            self.get_logger().error(f'Gains parse error: {e}')

    # ── Control loop ───────────────────────────────────────────────

    def _control_loop(self):
        with self._lock:
            if not self.enabled:
                return
            if self.takeoff_alt is None:
                return

            # Altitude relative to takeoff point
            rel_alt    = self.current_alt - self.takeoff_alt
            target     = self.target_alt
            kp, ki, kd = self.pid.kp, self.pid.ki, self.pid.kd

        pid_out  = self.pid.compute(target, rel_alt)
        throttle = int(THR_HOVER + pid_out)
        throttle = max(THR_MIN, min(THR_MAX, throttle))

        msg = PlutoMsg()
        msg.rc_roll     = NEUTRAL
        msg.rc_pitch    = NEUTRAL
        msg.rc_yaw      = NEUTRAL
        msg.rc_throttle = throttle
        msg.rc_aux1     = NEUTRAL
        msg.rc_aux2     = 1000
        msg.rc_aux3     = NEUTRAL
        msg.rc_aux4     = NEUTRAL
        msg.command_type = 0
        self.cmd_pub.publish(msg)

    # ── State publisher ────────────────────────────────────────────

    def _publish_state(self):
        with self._lock:
            ref  = self.takeoff_alt if self.takeoff_alt is not None else 0.0
            rel  = self.current_alt - ref
            err  = self.target_alt - rel
            state = {
                'enabled':      self.enabled,
                'target_alt':   round(self.target_alt, 3),
                'current_alt':  round(rel, 3),
                'raw_alt':      round(self.current_alt, 3),
                'error':        round(err, 3),
                'integral':     round(self.pid._integral, 3),
                'roll_deg':     round(self.current_roll, 2),
                'pitch_deg':    round(self.current_pitch, 2),
                'kp':           self.pid.kp,
                'ki':           self.pid.ki,
                'kd':           self.pid.kd,
                'thr_hover':    THR_HOVER,
            }
        msg = String()
        msg.data = json.dumps(state)
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AltitudeHoldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
pluto_tf_broadcaster.py  (jitter-fixed)
========================================
Fixes applied:
  1. QoS: BEST_EFFORT + depth=1 to match plutonode publisher (no queue buildup)
  2. Low-pass filter on quaternion (alpha=0.15) to smooth raw IMU noise
  3. Altitude low-pass filter (alpha=0.1) to damp barometer spikes
  4. TF broadcast driven by IMU callback (not a separate timer) — stays in sync
  5. world→odom static TF re-published every 5 s to survive RViz restarts
"""

import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def _slerp_scalar(a, b, alpha):
    """Simple linear blend then normalise — sufficient for small inter-frame angles."""
    x = a[0] + alpha * (b[0] - a[0])
    y = a[1] + alpha * (b[1] - a[1])
    z = a[2] + alpha * (b[2] - a[2])
    w = a[3] + alpha * (b[3] - a[3])
    n = math.sqrt(x*x + y*y + z*z + w*w) or 1.0
    return x/n, y/n, z/n, w/n


class PlutoTFBroadcaster(Node):

    def __init__(self):
        super().__init__('pluto_tf_broadcaster')

        # ── URDF ──────────────────────────────────────────────────
        urdf_path = os.path.join(
            os.path.dirname(__file__), '..', 'urdf', 'pluto.urdf'
        )
        if not os.path.exists(urdf_path):
            from ament_index_python.packages import get_package_share_directory
            urdf_path = os.path.join(
                get_package_share_directory('pluto_dashboard'),
                'urdf', 'pluto.urdf'
            )

        try:
            with open(urdf_path, 'r') as f:
                urdf_xml = f.read()
            desc_pub = self.create_publisher(String, '/robot_description', 10)
            msg = String()
            msg.data = urdf_xml
            self.create_timer(1.0, lambda: desc_pub.publish(msg))
            self.get_logger().info(f'URDF loaded from {urdf_path}')
        except FileNotFoundError:
            self.get_logger().error(f'URDF not found at {urdf_path}.')

        # ── TF broadcasters ────────────────────────────────────────
        self._tf_bc     = TransformBroadcaster(self)
        self._static_bc = StaticTransformBroadcaster(self)
        self._publish_static_world_odom()
        # Re-publish static TF every 5 s so RViz restarts don't lose it
        self.create_timer(5.0, self._publish_static_world_odom)

        # ── Filtered state ─────────────────────────────────────────
        # Quaternion low-pass: alpha=0.15 → ~6-7 frame lag at 150 Hz = ~45 ms
        # Reduces high-freq IMU jitter without killing responsiveness
        self._ALPHA_Q   = 0.15
        self._ALPHA_ALT = 0.10   # altitude barometer is noisier, smoother

        self._qx = 0.0
        self._qy = 0.0
        self._qz = 0.0
        self._qw = 1.0
        self._altitude  = 0.0
        self._got_imu   = False
        self._got_alt   = False

        # ── QoS: BEST_EFFORT depth=1 to match plutonode publisher ──
        # Using RELIABLE here causes queue buildup → delayed/jerky transforms
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ────────────────────────────────────────────
        # TF is broadcast inside _imu_cb so it stays phase-locked to IMU data
        self.create_subscription(Imu,   '/pluto/imu',      self._imu_cb, best_effort_qos)
        self.create_subscription(Range, '/pluto/altitude', self._alt_cb, best_effort_qos)

        self.get_logger().info(
            'PlutoTFBroadcaster started (jitter-fixed). '
            'Waiting for /pluto/imu and /pluto/altitude ...'
        )

    # ── Static world → odom ────────────────────────────────────────

    def _publish_static_world_odom(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id  = 'odom'
        t.transform.rotation.w = 1.0
        self._static_bc.sendTransform(t)

    # ── Sensor callbacks ───────────────────────────────────────────

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        raw = (q.x, q.y, q.z, q.w)

        if not self._got_imu:
            # Initialise filter with first reading — no jump on startup
            self._qx, self._qy, self._qz, self._qw = raw
            self._got_imu = True
            self.get_logger().info('IMU data received — attitude broadcasting active.')
        else:
            # Low-pass blend toward new measurement
            self._qx, self._qy, self._qz, self._qw = _slerp_scalar(
                (self._qx, self._qy, self._qz, self._qw),
                raw,
                self._ALPHA_Q,
            )

        # Broadcast TF immediately from this callback — no extra timer lag
        self._broadcast(msg.header.stamp)

    def _alt_cb(self, msg: Range):
        raw_alt = max(0.0, float(msg.range))
        if not self._got_alt:
            self._altitude = raw_alt
            self._got_alt  = True
            self.get_logger().info('Altitude data received — Z axis broadcasting active.')
        else:
            self._altitude += self._ALPHA_ALT * (raw_alt - self._altitude)

    # ── TF broadcast ───────────────────────────────────────────────

    def _broadcast(self, stamp):
        t = TransformStamped()
        t.header.stamp    = stamp      # use IMU timestamp, not node clock
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self._altitude

        t.transform.rotation.x = self._qx
        t.transform.rotation.y = self._qy
        t.transform.rotation.z = self._qz
        t.transform.rotation.w = self._qw

        self._tf_bc.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PlutoTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

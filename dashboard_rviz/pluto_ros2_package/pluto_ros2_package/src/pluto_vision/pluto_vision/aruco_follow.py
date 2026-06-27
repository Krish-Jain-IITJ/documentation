"""ArUco follower: subscribes to /plutocamera/image_raw, detects markers,
publishes /drone_command with proportional roll/pitch to keep the marker
centered in the frame. Throttle stays at neutral — altitude hold is the
firmware's job.

Enable with `--ros-args -p enabled:=true` or flip the `enabled` parameter at
runtime. Defaults to DISABLED so accidentally running this node can't hijack
manual flight.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from custom_msgs.msg import PlutoMsg

try:
    import cv2
    from cv_bridge import CvBridge
except ImportError as e:  # pragma: no cover
    raise SystemExit(f'pluto_vision needs cv_bridge + cv2 installed: {e}')


NEUTRAL = 1500


def _clip(x: float, lo: int = 1000, hi: int = 2000) -> int:
    return max(lo, min(hi, int(round(x))))


class ArucoFollow(Node):
    def __init__(self):
        super().__init__('pluto_aruco_follow')
        # Runtime-togglable: keeps the node harmless on accidental launch.
        self.declare_parameter('enabled', False)
        self.declare_parameter('dict_id', cv2.aruco.DICT_4X4_50)
        # Gain: how hard to push RC per pixel of error. ~200 units across a
        # half-frame puts the stick at limits when the marker is edge-of-view.
        self.declare_parameter('gain_px_to_rc', 1.5)
        # Marker ID to follow (-1 = any marker).
        self.declare_parameter('marker_id', -1)

        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.bridge = CvBridge()
        self.create_subscription(Image, '/plutocamera/image_raw', self._on_image, cam_qos)
        self.pub_cmd = self.create_publisher(PlutoMsg, '/drone_command', 10)

        self._aruco_dict = cv2.aruco.getPredefinedDictionary(
            int(self.get_parameter('dict_id').value))
        # Newer OpenCV (≥4.7) uses ArucoDetector; older uses detectMarkers() free func.
        if hasattr(cv2.aruco, 'ArucoDetector'):
            params = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, params)
        else:
            self._detector = None

        self.get_logger().info('pluto_aruco_follow ready (enabled=False by default).')

    def _detect(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._aruco_dict)
        return corners, ids

    def _on_image(self, msg: Image):
        if not bool(self.get_parameter('enabled').value):
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return

        corners, ids = self._detect(bgr)
        if ids is None or len(ids) == 0:
            # Marker lost: center sticks so the drone stops drifting.
            self._publish_rc(NEUTRAL, NEUTRAL)
            return

        want = int(self.get_parameter('marker_id').value)
        idx = 0
        if want >= 0:
            matches = [i for i, mid in enumerate(ids.flatten().tolist()) if mid == want]
            if not matches:
                self._publish_rc(NEUTRAL, NEUTRAL)
                return
            idx = matches[0]

        pts = corners[idx][0]           # 4× (x, y)
        cx = float(pts[:, 0].mean())
        cy = float(pts[:, 1].mean())
        h, w = bgr.shape[:2]
        err_x = cx - w / 2.0            # +ve = marker right of center
        err_y = cy - h / 2.0            # +ve = marker below center (image y down)

        gain = float(self.get_parameter('gain_px_to_rc').value)
        # Right (err_x > 0) → roll right (RC > 1500).
        # Marker below (err_y > 0) → nose down = pitch forward (RC < 1500).
        roll  = _clip(NEUTRAL + err_x * gain)
        pitch = _clip(NEUTRAL - err_y * gain)
        self._publish_rc(roll, pitch)

    def _publish_rc(self, roll: int, pitch: int):
        msg = PlutoMsg()
        msg.rc_roll     = roll
        msg.rc_pitch    = pitch
        msg.rc_yaw      = NEUTRAL
        msg.rc_throttle = NEUTRAL   # altitude hold assumed; do NOT push throttle here
        msg.rc_aux1 = NEUTRAL; msg.rc_aux2 = 1000; msg.rc_aux3 = 1000; msg.rc_aux4 = 1500
        msg.pluto_index = 0
        msg.command_type = 0
        msg.is_auto_pilot_on = True
        self.pub_cmd.publish(msg)


def main():
    rclpy.init()
    node = ArucoFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

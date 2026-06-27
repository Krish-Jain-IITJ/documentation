"""Pluto Dashboard — a ROS 2 node that:

- Acts as the server for service `pluto_service` (the plutonode driver
  sends telemetry as the REQUEST fields; we reply with RC values).
- Publishes to `/drone_command` (PlutoMsg) at 20 Hz with the latest user RC.
- Serves a Flask web UI on http://localhost:5050.
- Persists a CSV log to ros2_ws/logs/.
"""

from __future__ import annotations

import csv
import os
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int16MultiArray
import math
from sensor_msgs.msg import Image, BatteryState, Imu, Range
from custom_msgs.msg import PlutoMsg
from custom_msgs.srv import PlutoPilot

from flask import Flask, Response, jsonify, render_template, request, send_from_directory
import numpy as np

# Camera subscriber deps (optional — dashboard still runs if missing).
try:
    import cv2  # noqa: F401
    from cv_bridge import CvBridge
    _CAMERA_OK = True
except Exception as _e:
    CvBridge = None
    _CAMERA_OK = False
    _CAMERA_ERR = str(_e)

# MediaPipe runs in a separate venv subprocess (~/mediapipe_env) to avoid
# NumPy version conflicts with the ROS/OpenCV environment (NumPy 2.x).
# _HANDS_WORKER_SCRIPT is installed alongside dashboard_node.py.
import subprocess as _subprocess
import struct as _struct
import os as _os

_VENV_PYTHON = _os.path.expanduser('~/mediapipe_env/bin/python3')
_HANDS_WORKER = _os.path.join(_os.path.dirname(__file__), 'hands_worker.py')
_MP_OK = _os.path.isfile(_VENV_PYTHON) and _os.path.isfile(_HANDS_WORKER)


# ── Constants ─────────────────────────────────────────────────────────
NEUTRAL = 1500
LOG_DIR = Path.cwd() / "logs"
LOG_DIR.mkdir(exist_ok=True)


class DashboardNode(Node):
    def __init__(self):
        super().__init__('pluto_dashboard')

        # User RC state (driven by HTTP API or service replies)
        self.rc = {
            'roll': NEUTRAL, 'pitch': NEUTRAL,
            'yaw': NEUTRAL, 'throttle': NEUTRAL,
            'aux1': NEUTRAL, 'aux2': 1000,
            'aux3': 1000, 'aux4': 1000,
        }
        self.command_type = 0      # 0 none, 1 takeoff, 2 land
        self.armed = False         # purely a display flag

        # Latest telemetry from plutonode (received via service request)
        self.tele = {
            'battery': 0.0, 'current_ma': 0, 'mah_drawn': 0,
            'mah_remain': 0, 'soc': 0, 'auto_land': 0,
            'alt': 0.0,
            'roll': 0, 'pitch': 0, 'yaw': 0,
            'acc_x': 0.0, 'acc_y': 0.0, 'acc_z': 0.0,
            'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0,
            'mag_x': 0.0, 'mag_y': 0.0, 'mag_z': 0.0,
        }
        self.tele_count = 0

        # Logging state
        self.logging = False
        self.log_file = None
        self.log_rows = 0
        self._log_fp = None
        self._log_writer = None
        self._log_lock = threading.Lock()

        # ROS interfaces
        self.pub = self.create_publisher(PlutoMsg, '/drone_command', 10)
        self.pub_calib_acc = self.create_publisher(Empty, '/pluto/calibrate_acc', 10)
        self.pub_calib_mag = self.create_publisher(Empty, '/pluto/calibrate_mag', 10)
        self.pub_eeprom    = self.create_publisher(Empty, '/pluto/eeprom_write', 10)
        self.pub_motor     = self.create_publisher(Int16MultiArray, '/pluto/motor_test', 10)
        self.pub_acc_trim  = self.create_publisher(Int16MultiArray, '/pluto/acc_trim', 10)
        self.pub_set_pid   = self.create_publisher(Int16MultiArray, '/pluto/set_pid', 10)
        # Legacy pluto_service server kept for backward compat with old driver
        # builds. Current plutonode doesn't call it.
        self.srv = self.create_service(PlutoPilot, 'pluto_service', self._on_service)
        self.timer = self.create_timer(0.05, self._tick)   # 20 Hz publish loop

        # Telemetry via standard sensor_msgs topics (best-effort depth=1 to
        # match plutonode's publisher QoS and always see the latest sample).
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        tele_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(Imu,          '/pluto/imu',      self._on_imu,      tele_qos)
        self.create_subscription(BatteryState, '/pluto/battery',  self._on_battery,  tele_qos)
        self.create_subscription(Range,        '/pluto/altitude', self._on_altitude, tele_qos)

        # Camera subscriber (optional). Listens to the pluto_cam_ros2 publisher.
        self._cam_jpeg: bytes | None = None
        self._cam_lock = threading.Lock()
        self._cam_count = 0
        # Vision mode: None | 'aruco' | 'hands'
        self._vision_mode: str | None = None
        # MediaPipe worker subprocess (lazily started, persistent)
        self._hands_proc = None
        self._hands_lock = threading.Lock()
        # Non-blocking hands pipeline: background thread processes one frame
        # at a time; camera callback never waits for MediaPipe.
        self._hands_busy = False          # True while worker is processing
        self._hands_busy_lock = threading.Lock()

        # ── ArUco auto-hover pilot ─────────────────────────────────
        # Stores latest detected marker info: (cx_norm, cy_norm, area_norm)
        # cx/cy in [-1,1] relative to frame centre; area_norm in [0,1].
        self._aruco_detection = None   # None = no marker seen
        self._aruco_det_lock  = threading.Lock()
        self._aruco_pilot_on  = False  # True once pilot loop is running
        self._aruco_took_off  = False  # True once auto-takeoff has fired
        self._aruco_marker_seen = False  # True once a marker has been detected since mode was enabled

        # ── Lucas-Kanade optical flow state ───────────────────────────
        self._lk_old_gray: np.ndarray | None = None   # previous grayscale frame
        self._lk_p0:       np.ndarray | None = None   # tracked corner points
        self._lk_mask:     np.ndarray | None = None   # persistent trail canvas
        self._lk_colors  = np.random.randint(0, 255, (100, 3))
        self._lk_feature_params = dict(
            maxCorners   = 100,
            qualityLevel = 0.3,
            minDistance  = 7,
            blockSize    = 7,
        )
        self._lk_params = dict(
            winSize  = (15, 15),
            maxLevel = 2,
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )

        if _CAMERA_OK:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            self._bridge = CvBridge()
            # BestEffort + depth=1: for high-rate image topics, we want the
            # latest frame only. Reliable + queue=5 piles up old frames under
            # load and latency compounds.
            cam_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=1,
            )
            self.create_subscription(Image, '/plutocamera/image_raw', self._on_camera_frame, cam_qos)
            self.get_logger().info('camera: subscribed to /plutocamera/image_raw (best-effort)')
        else:
            self._bridge = None
            self.get_logger().warn(f'camera disabled (cv_bridge/cv2 missing: {_CAMERA_ERR})')

        self.get_logger().info('pluto_dashboard ready — http://localhost:5050')

    def _on_imu(self, msg: Imu):
        """Populate attitude / accel / gyro from the IMU topic.

        plutonode publishes orientation as a quaternion; we decode to Euler for
        display. Accel is already m/s² and gyro is rad/s (ROS REP-145), so we
        convert accel → g and gyro → deg/s to keep CSV/UI units consistent with
        before. Runs at ~100 Hz → CSV logged every 100th call = 1 Hz.
        """
        self.tele_count += 1
        if self.logging and self.tele_count % 100 == 0:
            self._log_sample()
        # Quaternion → Euler (ZYX convention, roll-pitch-yaw)
        qw, qx, qy, qz = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
        # pitch (y-axis rotation)
        sinp = 2.0 * (qw * qy - qz * qx)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        G = 9.80665
        self.tele.update({
            'roll':   round(roll, 1),
            'pitch':  round(pitch, 1),
            'yaw':    int(yaw),
            'acc_x':  round(msg.linear_acceleration.x / G, 3),
            'acc_y':  round(msg.linear_acceleration.y / G, 3),
            'acc_z':  round(msg.linear_acceleration.z / G, 3),
            'gyro_x': round(math.degrees(msg.angular_velocity.x), 2),
            'gyro_y': round(math.degrees(msg.angular_velocity.y), 2),
            'gyro_z': round(math.degrees(msg.angular_velocity.z), 2),
        })

    def _on_altitude(self, msg: Range):
        self.tele['alt'] = round(msg.range, 2)

    def _on_battery(self, msg: BatteryState):
        # BatteryState current/charge/capacity are SI (A, Ah). Store as mA/mAh.
        self.tele['battery']    = round(msg.voltage, 2)
        self.tele['current_ma'] = int(round(msg.current * 1000))
        self.tele['mah_drawn']  = int(round(msg.charge  * 1000))
        self.tele['mah_remain'] = int(round(msg.capacity * 1000))
        self.tele['soc']        = int(round(msg.percentage * 100))
        self.tele['auto_land']  = 1 if msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_DEAD else 0

    def _on_camera_frame(self, msg: Image):
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            mode = self._vision_mode

            if mode == 'aruco':
                bgr = self._process_aruco(bgr)
                ok, jpg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 60])
                if not ok:
                    return
                with self._cam_lock:
                    self._cam_jpeg = jpg.tobytes()
                    self._cam_count += 1
            elif mode == 'hands':
                # Publish raw frame immediately so the stream never stalls.
                ok, jpg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 60])
                if ok:
                    with self._cam_lock:
                        self._cam_jpeg = jpg.tobytes()
                        self._cam_count += 1
                # Fire hand processing in background only if worker is idle.
                with self._hands_busy_lock:
                    if not self._hands_busy:
                        self._hands_busy = True
                        bgr_copy = bgr.copy()
                        threading.Thread(
                            target=self._hands_worker_thread,
                            args=(bgr_copy,),
                            daemon=True,
                        ).start()
            else:
                ok, jpg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 60])
                if not ok:
                    return
                with self._cam_lock:
                    self._cam_jpeg = jpg.tobytes()
                    self._cam_count += 1
        except Exception as e:
            self.get_logger().warn(f'camera frame decode failed: {e}')

    def _hands_worker_thread(self, bgr):
        """Background thread: send one frame to MediaPipe worker, update cam_jpeg."""
        try:
            result_bgr = self._process_hands(bgr)
            ok, jpg = cv2.imencode('.jpg', result_bgr, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if ok:
                with self._cam_lock:
                    self._cam_jpeg = jpg.tobytes()
                    self._cam_count += 1
        except Exception as e:
            self.get_logger().warn(f'hands worker thread error: {e}')
        finally:
            with self._hands_busy_lock:
                self._hands_busy = False

    # ─── Porous green ArUco overlay ──────────────────────────────────
    def _process_aruco(self, bgr):
        """Detect ArUco markers, draw overlay, and update pilot detection state."""
        try:
            h, w = bgr.shape[:2]
            aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
            aruco_params = cv2.aruco.DetectorParameters()
            detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            corners, ids, _ = detector.detectMarkers(bgr)
            detected = None
            if ids is not None:
                for corner in corners:
                    pts = corner.reshape((-1, 1, 2)).astype(int)
                    overlay = bgr.copy()
                    cv2.fillPoly(overlay, [pts], (0, 200, 80))
                    cv2.addWeighted(overlay, 0.35, bgr, 0.65, 0, bgr)
                    cv2.polylines(bgr, [pts], True, (0, 255, 80), 2)
                    cx = int(corner[0, :, 0].mean())
                    cy = int(corner[0, :, 1].mean())
                    cv2.putText(bgr, f'ID:{ids[0][0]}', (cx-20, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 80), 2)
                # Use first detected marker for pilot
                c = corners[0]
                cx_px = float(c[0, :, 0].mean())
                cy_px = float(c[0, :, 1].mean())
                # Marker side length in pixels → proxy for distance
                side  = float(np.linalg.norm(c[0, 0] - c[0, 1]))
                # Normalise: cx/cy in [-1,1], area_norm in [0,1]
                cx_n = (cx_px - w / 2) / (w / 2)
                cy_n = (cy_px - h / 2) / (h / 2)
                area_n = min(1.0, (side * side) / (w * h))
                detected = (cx_n, cy_n, area_n)
            # Update pilot detection (pilot loop already started by vision mode button)
            with self._aruco_det_lock:
                self._aruco_detection = detected

            # First detection since ArUco mode was enabled → fire takeoff + pilot
            if (detected is not None and self._vision_mode == 'aruco'
                    and not self._aruco_marker_seen):
                self._aruco_marker_seen = True
                self.get_logger().warn('ArUco marker detected: firing takeoff')

                def _takeoff_then_pilot():
                    _delayed_takeoff()
                    time.sleep(self._TAKEOFF_WAIT)
                    if self._vision_mode == 'aruco' and not self._aruco_pilot_on:
                        self._aruco_pilot_on = True
                        self._aruco_pilot_loop()

                threading.Thread(target=_takeoff_then_pilot, daemon=True).start()
        except Exception as e:
            self.get_logger().warn(f'aruco processing error: {e}')

        # ── Lucas-Kanade optical flow on top of ArUco overlay ────────
        bgr = self._process_lk(bgr)

        return bgr

    # ─── Lucas-Kanade sparse optical flow overlay ────────────────────
    def _process_lk(self, bgr: np.ndarray) -> np.ndarray:
        """Compute Lucas-Kanade sparse optical flow and draw colour trails
        on top of the incoming bgr frame (in-place safe — we only write to
        the persistent mask and then cv2.add it onto bgr)."""
        try:
            # Guard: reset if frame size changed (e.g. resolution switch)
            if (self._lk_mask is not None and
                    self._lk_mask.shape != bgr.shape):
                self._lk_old_gray = None
                self._lk_p0       = None
                self._lk_mask     = None

            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

            # First frame or full reset — initialise, nothing to draw yet
            if self._lk_old_gray is None or self._lk_p0 is None or len(self._lk_p0) == 0:
                self._lk_old_gray = gray
                self._lk_p0 = cv2.goodFeaturesToTrack(
                    gray, mask=None, **self._lk_feature_params)
                self._lk_mask = np.zeros_like(bgr)
                return bgr

            # Calculate sparse optical flow
            p1, st, _ = cv2.calcOpticalFlowPyrLK(
                self._lk_old_gray, gray, self._lk_p0, None, **self._lk_params)

            if p1 is None or st is None:
                # Tracking failed entirely — full reset next frame
                self._lk_old_gray = None
                self._lk_p0       = None
                return bgr

            good_new = p1[st == 1]
            good_old = self._lk_p0[st == 1]

            # Draw trails on the persistent mask, dots on live frame
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel().astype(int)
                c, d = old.ravel().astype(int)
                col  = self._lk_colors[i % len(self._lk_colors)].tolist()
                self._lk_mask = cv2.line(self._lk_mask, (a, b), (c, d), col, 2)
                bgr            = cv2.circle(bgr, (a, b), 4, col, -1)

            # Fade old trails so they don't permanently clutter the frame
            self._lk_mask = (self._lk_mask * 0.93).astype(np.uint8)

            bgr = cv2.add(bgr, self._lk_mask)

            # Re-seed features when too few points survive tracking
            if len(good_new) < 10:
                self._lk_p0   = cv2.goodFeaturesToTrack(
                    gray, mask=None, **self._lk_feature_params)
                self._lk_mask = np.zeros_like(bgr)
            else:
                self._lk_p0 = good_new.reshape(-1, 1, 2)

            self._lk_old_gray = gray.copy()

        except Exception as e:
            self.get_logger().warn(f'LK optical flow error: {e}')

        return bgr

    # ── ArUco auto-hover pilot loop ───────────────────────────────────────
    # Runs in a daemon thread once the first marker is detected.
    # Phase 1: auto-takeoff (fires _delayed_takeoff once).
    # Phase 2: PD control on roll (centering) + small yaw correction + pitch
    #          (distance) + throttle bias to hold apparent marker size
    #          (≈ constant altitude).
    # Phase 3: if marker lost, holds last RC values (neutral = hover in place).
    _PILOT_DT       = 0.005   # 20 Hz control loop
    _PILOT_KP_HORIZ = 60     # roll proportional gain (left/right centering)
    _PILOT_KD_HORIZ = 10     # roll derivative gain
    _PILOT_KP_YAW   = 30     # small secondary yaw correction (keep marker facing camera)
    _PILOT_KD_YAW   = 5      # yaw derivative gain
    _TARGET_AREA    = 0.0033 # target area for ~50 cm hover with 5cm marker
    _PILOT_KP_ALT   = 50     # throttle proportional gain
    _PILOT_KD_ALT   = 10     # throttle derivative gain
    _TAKEOFF_WAIT   = 3.0    # seconds after takeoff before PD engages
    _HOVER_THROTTLE = 1600   # base throttle to maintain altitude (tune up if drone sinks)

    def _aruco_pilot_loop(self):
        """Background daemon: PD hover on ArUco marker.
        Takeoff is triggered manually via the ArUco ON button — this loop
        only handles stabilisation once the drone is already in the air.
        """
        self.get_logger().info('ArUco pilot: waiting for marker...')
        # Wait briefly for first detection before engaging PD
        for _ in range(40):   # up to 2 s
            with self._aruco_det_lock:
                if self._aruco_detection is not None:
                    break
            time.sleep(0.05)

        # ── PD hover loop ────────────────────────────────────────────
        prev_ex = prev_ey = prev_ea = 0.0
        marker_lost_count = 0
        self.get_logger().info('ArUco pilot: engaging PD hover')
        while self._vision_mode == 'aruco':
            with self._aruco_det_lock:
                det = self._aruco_detection

            if det is not None:
                cx_n, cy_n, area_n = det
                # Front camera axes:
                #   cx_n > 0  → marker RIGHT  → roll right (and yaw right slightly)
                #   cy_n > 0  → marker BELOW  → climb (increase throttle)
                #   area_n > TARGET → too CLOSE → back off (negative pitch)
                ex =  cx_n                        # horizontal error (left/right)
                ey = -cy_n                        # altitude error (flip: below = climb)
                ea =  area_n - self._TARGET_AREA  # distance error

                # PD terms (derivative smoothed by small dt)
                d_ex = (ex - prev_ex) / self._PILOT_DT
                d_ey = (ey - prev_ey) / self._PILOT_DT
                d_ea = (ea - prev_ea) / self._PILOT_DT

                # roll handles left/right centering (lateral translation —
                # much more direct than yawing toward the marker)
                roll_adj     = int(self._PILOT_KP_HORIZ * ex + self._PILOT_KD_HORIZ * d_ex)
                # small secondary yaw correction so the camera keeps facing
                # the marker as the drone strafes
                yaw_adj      = int(self._PILOT_KP_YAW   * ex + self._PILOT_KD_YAW   * d_ex)
                throttle_adj = int(self._PILOT_KP_ALT   * ey + self._PILOT_KD_ALT   * d_ey)
                pitch_adj    = int(-self._PILOT_KP_HORIZ * ea - self._PILOT_KD_HORIZ * d_ea)

                # Base hover throttle + correction; clamp to safe range
                self.rc['roll']     = max(1600, min(1650, NEUTRAL + roll_adj))
                self.rc['yaw']      = max(1600, min(1600, NEUTRAL + yaw_adj))
                self.rc['throttle'] = max(1600, min(1750, self._HOVER_THROTTLE + throttle_adj))
                self.rc['pitch']    = max(1600, min(1650, NEUTRAL + pitch_adj))

                prev_ex, prev_ey, prev_ea = ex, ey, ea
                marker_lost_count = 0
            else:
                # Marker lost — hold last throttle, neutralise attitude
                # Only reset prev errors to avoid derivative spike on re-detection
                marker_lost_count += 1
                if marker_lost_count > 10:   # >0.5 s lost → full neutral
                    self.rc['roll']  = NEUTRAL
                    self.rc['pitch'] = NEUTRAL
                    self.rc['yaw']   = NEUTRAL
                prev_ex = prev_ey = prev_ea = 0.0

            time.sleep(self._PILOT_DT)

        # Vision mode changed — neutralise pilot RC
        self.rc['roll']  = NEUTRAL
        self.rc['pitch'] = NEUTRAL
        self.rc['yaw']   = NEUTRAL
        self._aruco_pilot_on  = False
        self._aruco_took_off  = False
        self.get_logger().info('ArUco pilot: stopped (vision mode changed)')



    # ─── Porous green hand overlay (MediaPipe subprocess) ───────────────
    def _process_hands(self, bgr):
        """Send BGR frame to the mediapipe venv worker; receive annotated JPEG."""
        if not _MP_OK:
            return bgr
        try:
            # Lazy-start the worker process
            with self._hands_lock:
                if self._hands_proc is None or self._hands_proc.poll() is not None:
                    self._hands_proc = _subprocess.Popen(
                        [_VENV_PYTHON, _HANDS_WORKER],
                        stdin=_subprocess.PIPE,
                        stdout=_subprocess.PIPE,
                        stderr=_subprocess.DEVNULL,
                    )
                proc = self._hands_proc

            # Encode current frame to JPEG and send to worker
            ok, jpg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                return bgr
            jpg_bytes = jpg.tobytes()
            proc.stdin.write(_struct.pack('>I', len(jpg_bytes)))
            proc.stdin.write(jpg_bytes)
            proc.stdin.flush()

            # Read annotated JPEG back from worker (with 2 s timeout so the
            # background thread never hangs indefinitely if the worker stalls).
            import select as _select
            def _read_exact(fd, n, timeout=2.0):
                buf = b''
                import time as _time
                deadline = _time.monotonic() + timeout
                while len(buf) < n:
                    rem = deadline - _time.monotonic()
                    if rem <= 0:
                        return None
                    ready, _, _ = _select.select([fd], [], [], rem)
                    if not ready:
                        return None
                    chunk = fd.read(n - len(buf))
                    if not chunk:
                        return None
                    buf += chunk
                return buf

            hdr = _read_exact(proc.stdout, 4)
            if hdr is None or len(hdr) < 4:
                return bgr
            n = _struct.unpack('>I', hdr)[0]
            out_bytes = _read_exact(proc.stdout, n)
            if out_bytes is None:
                return bgr

            arr = np.frombuffer(out_bytes, dtype=np.uint8)
            result = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            return result if result is not None else bgr

        except Exception as e:
            self.get_logger().warn(f'hand worker error: {e}')
            with self._hands_lock:
                self._hands_proc = None   # will restart on next frame
            return bgr

    # ─── Legacy ROS service handler ──────────────────────────────────
    # Historical path: plutonode → pluto_service (req=telemetry, resp=RC).
    # Current plutonode doesn't call this — telemetry flows via /pluto/imu +
    # /pluto/battery + /pluto/altitude. Kept so older driver builds don't
    # log "service unavailable" on startup.
    def _on_service(self, req: PlutoPilot.Request, resp: PlutoPilot.Response):
        self.tele_count += 1  # keeps the "connected" indicator lit
        # Log every Nth (20 Hz → ~1 Hz file write)
        if self.logging and self.tele_count % 20 == 0:
            self._log_sample()
        # Reply with current RC (for parity; actual control flows via /drone_command)
        resp.rc_roll = int(self.rc['roll'])
        resp.rc_pitch = int(self.rc['pitch'])
        resp.rc_yaw = int(self.rc['yaw'])
        resp.rc_throttle = int(self.rc['throttle'])
        resp.rc_aux1 = int(self.rc['aux1'])
        resp.rc_aux2 = int(self.rc['aux2'])
        resp.rc_aux3 = int(self.rc['aux3'])
        resp.rc_aux4 = int(self.rc['aux4'])
        return resp

    # ─── 20 Hz publish loop ──────────────────────────────────────────
    def _tick(self):
        """Just publish whatever is in self.rc. Arm/disarm handlers latch
        the right values there; firmware AUX4 switch maintains arm state."""
        msg = PlutoMsg()
        msg.rc_roll     = int(self.rc['roll'])
        msg.rc_pitch    = int(self.rc['pitch'])
        msg.rc_yaw      = int(self.rc['yaw'])
        msg.rc_throttle = int(self.rc['throttle'])
        msg.rc_aux1     = int(self.rc['aux1'])
        msg.rc_aux2     = int(self.rc['aux2'])
        msg.rc_aux3     = int(self.rc['aux3'])
        msg.rc_aux4     = int(self.rc['aux4'])
        msg.pluto_index = 0
        msg.command_type = int(self.command_type)
        msg.trim_roll = 0
        msg.trim_pitch = 0
        msg.is_auto_pilot_on = False

        self.pub.publish(msg)
        if self.command_type != 0:
            self.command_type = 0  # one-shot

    # ─── Logging ─────────────────────────────────────────────────────
    _RC_KEYS  = ('roll', 'pitch', 'yaw', 'throttle', 'aux1', 'aux2', 'aux3', 'aux4')
    _PID_AXES = ('roll', 'pitch', 'yaw')
    _PID_GAINS = ('p', 'i', 'd')

    def _log_header(self):
        cols = ["timestamp"]
        cols += list(self.tele.keys())
        cols += [f'rc_{k}' for k in self._RC_KEYS]
        cols += [f'pid_{a}_{g}' for a in self._PID_AXES for g in self._PID_GAINS]
        return cols

    def start_log(self) -> str:
        with self._log_lock:
            if self.logging:
                return self.log_file
            name = datetime.now().strftime("flight_%Y%m%d_%H%M%S.csv")
            path = LOG_DIR / name
            self._log_fp = open(path, "w", newline="")
            self._log_writer = csv.writer(self._log_fp)
            self._log_writer.writerow(self._log_header())
            self.logging = True
            self.log_file = name
            self.log_rows = 0
            self.get_logger().info(f'log started → {path}')
            return name

    def stop_log(self):
        with self._log_lock:
            if not self.logging:
                return
            self._log_fp.close()
            self._log_fp = None
            self._log_writer = None
            self.logging = False
            self.get_logger().info(f'log stopped after {self.log_rows} rows')

    # Per-key CSV precision. Missing key → int/passthrough.
    _LOG_PRECISION = {
        'battery': 2, 'alt': 2,
        'acc_x': 3, 'acc_y': 3, 'acc_z': 3,
        'gyro_x': 2, 'gyro_y': 2, 'gyro_z': 2,
        'mag_x': 1, 'mag_y': 1, 'mag_z': 1,
        'roll': 1, 'pitch': 1,   # stored as decidegrees int, but kept here for future
    }

    @staticmethod
    def _fmt(val, digits):
        if isinstance(val, float):
            return f'{val:.{digits}f}'
        return val

    def _log_sample(self):
        with self._log_lock:
            if self._log_writer is None:
                return
            ts = f'{time.time():.3f}'
            row = [ts]
            for k, v in self.tele.items():
                row.append(self._fmt(v, self._LOG_PRECISION.get(k, 3)))
            row += [self.rc[k] for k in self._RC_KEYS]
            row += [_pid_state[a][g] for a in self._PID_AXES for g in self._PID_GAINS]
            self._log_writer.writerow(row)
            self._log_fp.flush()
            self.log_rows += 1


# ── Process manager (launch helper scripts from the browser) ────────
# Repo root holds start_drone.sh / start_camera.sh.
REPO_ROOT = Path(__file__).resolve().parents[4] if (Path(__file__).resolve().parents[4] / 'start_drone.sh').exists() else Path.cwd().parent
import socket
import subprocess
from collections import deque


def _tcp_probe(host: str, port: int, timeout: float = 0.3) -> bool:
    """Quick TCP connect — used to detect which Wi-Fi AP we're on."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False

PROC_CONFIG = {
    'drone':  {'script': 'start_drone.sh',  'label': 'plutonode (flight)'},
    'camera': {'script': 'start_camera.sh', 'label': 'pluto_camera'},
    'vision': {'script': 'start_vision.sh', 'label': 'pluto_vision (ArUco, disabled)'},
}

class _Proc:
    def __init__(self, name, script, label):
        self.name = name
        self.script = script
        self.label = label
        self.proc: subprocess.Popen | None = None
        self.log = deque(maxlen=200)  # last 200 lines
        self._lock = threading.Lock()

    def start(self, cwd: Path):
        with self._lock:
            if self.proc and self.proc.poll() is None:
                return False, 'already running'
            script_path = cwd / self.script
            if not script_path.exists():
                return False, f'missing {script_path}'
            self.log.clear()
            self.proc = subprocess.Popen(
                ['/bin/bash', str(script_path)],
                cwd=str(cwd),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                start_new_session=True,  # so we can SIGTERM the whole group
            )
            threading.Thread(target=self._reader, daemon=True).start()
            return True, 'started'

    def stop(self):
        with self._lock:
            if not self.proc or self.proc.poll() is not None:
                return False, 'not running'
            try:
                os.killpg(os.getpgid(self.proc.pid), 15)  # SIGTERM group
            except ProcessLookupError:
                pass
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.proc.pid), 9)   # SIGKILL
            return True, 'stopped'

    def status(self):
        running = bool(self.proc and self.proc.poll() is None)
        return {
            'name': self.name, 'label': self.label,
            'running': running,
            'pid': self.proc.pid if running else None,
            'exit_code': None if running else (self.proc.returncode if self.proc else None),
        }

    def _reader(self):
        if not self.proc or not self.proc.stdout:
            return
        for line in iter(self.proc.stdout.readline, b''):
            try:
                self.log.append(line.decode(errors='replace').rstrip())
            except Exception:
                pass
        self.proc.stdout.close()


procs: dict[str, _Proc] = {k: _Proc(k, v['script'], v['label']) for k, v in PROC_CONFIG.items()}


# ── Flask app (one global, bound to node in main()) ─────────────────
app = Flask(__name__)
node: DashboardNode | None = None


@app.route('/')
def index():
    return render_template('index.html')


# ── Process endpoints ───────────────────────────────────────
@app.route('/api/proc/status')
def api_proc_status():
    return jsonify({k: p.status() for k, p in procs.items()})


@app.route('/api/network/probe')
def api_network_probe():
    """Detect connection mode.

    Pluto has two configurations:
      - **camera attached**: laptop joins WIFI-1080p-… (192.168.0.x). The
        camera module is the AP; it exposes video on 192.168.0.1:8065 and
        relays MSP to the flight controller on 192.168.0.1:9060. The drone's
        own AP (192.168.4.1) is OFF.
      - **no camera module**: laptop joins the drone's direct AP, FC at
        192.168.4.1:23. No camera.
    """
    drone_direct = _tcp_probe('192.168.4.1', 23)
    drone_via_cam = _tcp_probe('192.168.0.1', 9060)
    camera = _tcp_probe('192.168.0.1', 8065)  # plutocam CMD_PORT (stream is 7065)
    drone = drone_direct or drone_via_cam
    if camera and drone_via_cam:
        mode = 'camera'
    elif drone_direct:
        mode = 'drone'
    elif camera and not drone_via_cam:
        mode = 'camera-only'
    else:
        mode = 'none'
    return jsonify({
        'drone': drone, 'camera': camera,
        'drone_direct': drone_direct, 'drone_via_cam': drone_via_cam,
        'mode': mode,
    })


@app.route('/api/proc/start/<name>', methods=['POST'])
def api_proc_start(name):
    if name not in procs:
        return jsonify({'ok': False, 'error': 'unknown'}), 404
    ok, msg = procs[name].start(REPO_ROOT)
    return jsonify({'ok': ok, 'msg': msg})


@app.route('/api/proc/stop/<name>', methods=['POST'])
def api_proc_stop(name):
    if name not in procs:
        return jsonify({'ok': False, 'error': 'unknown'}), 404
    ok, msg = procs[name].stop()
    return jsonify({'ok': ok, 'msg': msg})


@app.route('/api/proc/log/<name>')
def api_proc_log(name):
    if name not in procs:
        return jsonify([]), 404
    return jsonify(list(procs[name].log))


@app.route('/api/telemetry')
def api_tele():
    # self.tele already holds engineering units (scaling done in _on_service).
    return jsonify({
        **node.tele,
        'battery_pct': max(0, min(100, int((node.tele['battery'] - 3.3) / (4.2 - 3.3) * 100))) if node.tele['battery'] else 0,
        'connected': node.tele_count > 0,
        'armed': node.armed,
        'dev_mode': False,
        'logging': node.logging,
        'log_file': node.log_file,
        'log_rows': node.log_rows,
        'tele_count': node.tele_count,
    })


# Flight commands — mirror plutocontrol exactly (latch RC values, don't revert)
@app.route('/api/arm', methods=['POST'])
def api_arm():
    node.rc['roll'] = NEUTRAL
    node.rc['pitch'] = NEUTRAL
    node.rc['yaw'] = NEUTRAL
    node.rc['throttle'] = 1000
    node.rc['aux4'] = 1500
    node.armed = True
    node.get_logger().warn('ARM: throttle=1000 AUX4=1500 (latched)')
    return jsonify({'ok': True})


@app.route('/api/disarm', methods=['POST'])
def api_disarm():
    node.rc['throttle'] = 1300
    node.rc['aux4'] = 1200
    node.armed = False
    node.get_logger().warn('DISARM: throttle=1300 AUX4=1200 (latched)')
    return jsonify({'ok': True})


@app.route('/api/box_arm', methods=['POST'])
def api_box_arm():
    node.rc['roll'] = NEUTRAL
    node.rc['pitch'] = NEUTRAL
    node.rc['yaw'] = NEUTRAL
    node.rc['throttle'] = 1800
    node.rc['aux4'] = 1500
    node.armed = True
    node.get_logger().warn('BOX ARM: throttle=1800 AUX4=1500 (latched)')
    return jsonify({'ok': True})


def _delayed_takeoff():
    """Stable take-off: disarm → zero sticks → box_arm → settle → cmd=1.

    Explicitly zeros roll/pitch/yaw before the takeoff command so a stale
    non-neutral stick value cannot cause the drone to drift forward on climb.
    """
    # 1. Disarm and neutralise all sticks.
    node.rc['throttle'] = 1300
    node.rc['aux4'] = 1200
    node.rc['roll'] = NEUTRAL
    node.rc['pitch'] = NEUTRAL
    node.rc['yaw'] = NEUTRAL
    time.sleep(0.6)        # firmware needs time to register the disarm

    # 2. Re-assert neutral (belt-and-suspenders).
    node.rc['roll'] = NEUTRAL
    node.rc['pitch'] = NEUTRAL
    node.rc['yaw'] = NEUTRAL

    # 3. Box-arm (throttle high + AUX4 mid = armed-angle mode).
    node.rc['throttle'] = 1800
    node.rc['aux4'] = 1500
    time.sleep(0.5)        # ~400 ms for firmware to accept the arm state

    # 4. Final stick zero before sending the autonomous takeoff command.
    node.rc['roll'] = NEUTRAL
    node.rc['pitch'] = NEUTRAL
    node.rc['yaw'] = NEUTRAL

    # 5. Issue takeoff command (one-shot, cleared in _tick).
    node.command_type = 1
    node.armed = True


@app.route('/api/takeoff', methods=['POST'])
def api_takeoff():
    node.get_logger().warn('TAKEOFF: disarm → box_arm → cmd=1')
    threading.Thread(target=_delayed_takeoff, daemon=True).start()
    return jsonify({'ok': True})


@app.route('/api/land', methods=['POST'])
def api_land():
    node.command_type = 2
    node.get_logger().warn('LAND: cmd=2')
    return jsonify({'ok': True})


# RC channels
@app.route('/api/rc', methods=['POST'])
def api_rc():
    data = request.get_json(silent=True) or {}
    for k in ('roll', 'pitch', 'yaw', 'throttle'):
        if k in data:
            node.rc[k] = max(1000, min(2000, int(data[k])))
    return jsonify({'ok': True})


@app.route('/api/neutral', methods=['POST'])
def api_neutral():
    for k in ('roll', 'pitch', 'yaw', 'throttle'):
        node.rc[k] = NEUTRAL
    return jsonify({'ok': True})


# Movement (one-shot delta on next publish)
_MOVE_DIRS = {
    'forward':  ('pitch', +200), 'backward': ('pitch', -200),
    'left':     ('roll',  -200), 'right':    ('roll',  +200),
    'left_yaw': ('yaw',   -200), 'right_yaw':('yaw',   +200),
    'up':       ('throttle', +200), 'down':   ('throttle', -200),
    'reset':    None,
}


@app.route('/api/move/<direction>', methods=['POST'])
def api_move(direction):
    cfg = _MOVE_DIRS.get(direction)
    if cfg is None and direction != 'reset':
        return jsonify({'ok': False, 'error': 'unknown'}), 400
    if direction == 'reset':
        for k in ('roll', 'pitch', 'yaw', 'throttle'):
            node.rc[k] = NEUTRAL
    else:
        axis, delta = cfg
        node.rc[axis] = max(1000, min(2000, NEUTRAL + delta))
    return jsonify({'ok': True})


# Calibration / trim — publish to plutonode which forwards via MSP.
@app.route('/api/calibrate/<which>', methods=['POST'])
def api_calib(which):
    if which == 'accelerometer':
        node.pub_calib_acc.publish(Empty())
    elif which == 'magnetometer':
        node.pub_calib_mag.publish(Empty())
    else:
        return jsonify({'ok': False, 'error': 'unknown'}), 400
    node.get_logger().info(f'CALIBRATE {which} → /pluto/calibrate_*')
    return jsonify({'ok': True})


@app.route('/api/trim', methods=['POST'])
def api_trim():
    data = request.get_json(silent=True) or {}
    r = max(-1000, min(1000, int(data.get('roll', 0))))
    p = max(-1000, min(1000, int(data.get('pitch', 0))))
    msg = Int16MultiArray()
    msg.data = [r, p]
    node.pub_acc_trim.publish(msg)
    node.get_logger().info(f'TRIM r={r} p={p} → /pluto/acc_trim')
    return jsonify({'ok': True})


# Dev mode = AUX2 switch on Pluto firmware (enables MSP debug stream).
# AUX2=1500 → ON, AUX2=1000 → OFF. Mirrors plutocontrol.Pluto.devOn/devOff.
@app.route('/api/dev/<state>', methods=['POST'])
def api_dev(state):
    on = (state == 'on')
    node.rc['aux2'] = 1500 if on else 1000
    node.get_logger().info(f'DEV mode {state.upper()}: AUX2={node.rc["aux2"]}')
    return jsonify({'ok': True})


# Motor test — publishes to /pluto/motor_test which plutonode forwards to
# MSP_SET_MOTOR. PROPS MUST BE OFF.
@app.route('/api/motor', methods=['POST'])
def api_motor():
    data = request.get_json(silent=True) or {}
    idx   = max(0, min(3, int(data.get('index', 0))))
    speed = max(1000, min(2000, int(data.get('speed', 1000))))
    motors = [1000, 1000, 1000, 1000]
    motors[idx] = speed
    msg = Int16MultiArray()
    msg.data = motors
    node.pub_motor.publish(msg)
    node.get_logger().warn(f'MOTOR M{idx}={speed} → /pluto/motor_test')
    return jsonify({'ok': True})


# Emergency stop
@app.route('/api/estop', methods=['POST'])
def api_estop():
    node.rc['roll'] = NEUTRAL
    node.rc['pitch'] = NEUTRAL
    node.rc['yaw'] = NEUTRAL
    node.rc['throttle'] = 1300
    node.rc['aux4'] = 1200
    node.command_type = 0
    node.armed = False
    node.get_logger().error('EMERGENCY STOP')
    return jsonify({'ok': True})


# Logging
@app.route('/api/log/start', methods=['POST'])
def api_log_start():
    return jsonify({'ok': True, 'file': node.start_log()})


@app.route('/api/log/stop', methods=['POST'])
def api_log_stop():
    node.stop_log()
    return jsonify({'ok': True})


@app.route('/api/log/list')
def api_log_list():
    files = []
    for p in sorted(LOG_DIR.glob("*.csv"), reverse=True):
        files.append({'name': p.name, 'size': p.stat().st_size, 'modified': p.stat().st_mtime})
    return jsonify(files)


@app.route('/logs/<name>')
def api_log_download(name):
    return send_from_directory(LOG_DIR, name, as_attachment=True)


# ── Camera ──────────────────────────────────────────────────
@app.route('/api/camera/status')
def api_camera_status():
    return jsonify({
        'available': _CAMERA_OK,
        'streaming': node._cam_jpeg is not None,
        'frames': node._cam_count,
    })


@app.route('/api/camera/snapshot')
def api_camera_snapshot():
    with node._cam_lock:
        jpg = node._cam_jpeg
    if not jpg:
        return ('no frame', 503)
    return Response(jpg, mimetype='image/jpeg')


@app.route('/api/camera/stream')
def api_camera_stream():
    """MJPEG multipart stream consumable by any <img> tag.

    Polls the latest frame at 60 Hz; pushes whenever the frame counter
    advances. Publisher is the actual rate limiter (camera FPS).
    """
    def gen():
        last = -1
        while True:
            with node._cam_lock:
                jpg = node._cam_jpeg
                count = node._cam_count
            if jpg and count != last:
                last = count
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n'
                       b'Content-Length: ' + str(len(jpg)).encode() + b'\r\n\r\n'
                       + jpg + b'\r\n')
            else:
                time.sleep(0.016)  # idle poll ~60 Hz
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')



# ── Vision Mode ──────────────────────────────────────────────────────────────
@app.route('/api/vision/mode', methods=['GET'])
def api_vision_mode_get():
    return jsonify({
        'mode': node._vision_mode,
        'marker_detected': node._aruco_detection is not None,
        'marker_seen': node._aruco_marker_seen,
        'pilot_on': node._aruco_pilot_on,
    })


@app.route('/api/vision/mode', methods=['POST'])
def api_vision_mode_set():
    data = request.get_json(silent=True) or {}
    mode = data.get('mode')  # 'aruco' | 'hands' | None
    if mode not in (None, 'aruco', 'hands'):
        return jsonify({'error': 'invalid mode'}), 400
    # Terminate hand worker subprocess when switching off hands
    if mode != 'hands':
        with node._hands_lock:
            if node._hands_proc and node._hands_proc.poll() is None:
                try:
                    node._hands_proc.stdin.close()
                    node._hands_proc.wait(timeout=2)
                except Exception:
                    node._hands_proc.kill()
            node._hands_proc = None
    node._vision_mode = mode
    # Reset ArUco pilot state whenever mode changes
    if mode != 'aruco':
        node._aruco_marker_seen = False
        node._aruco_took_off = False
        node._aruco_pilot_on = False
        with node._aruco_det_lock:
            node._aruco_detection = None
    else:
        # Arm waiting state — takeoff fires automatically once a marker is
        # detected (see _process_aruco's first-detection trigger).
        node._aruco_marker_seen = False
        node._aruco_took_off = False
        node._aruco_pilot_on = False
        with node._aruco_det_lock:
            node._aruco_detection = None
        node.get_logger().info('ArUco mode armed — waiting for marker...')
    return jsonify({
        'mode': node._vision_mode,
        'marker_detected': node._aruco_detection is not None,
        'marker_seen': node._aruco_marker_seen,
        'pilot_on': node._aruco_pilot_on,
    })


# ── CrazyRadio PA runtime connect / disconnect ───────────────────────────────
# The user can switch to CrazyRadio from the dashboard UI without restarting
# the launch file.  The bridge node (crazyflie_bridge) is already the cleanest
# path; this endpoint simply lets the dashboard start it on demand.
#
# Implementation: we just spawn / stop the crazyflie_bridge console_script in a
# subprocess the same way we handle start_drone.sh, but we build the CLI args
# dynamically from the request body.

_radio_proc: _Proc | None = None

@app.route('/api/radio/connect', methods=['POST'])
def api_radio_connect():
    global _radio_proc
    data = request.get_json(silent=True) or {}
    channel  = int(data.get('channel',  80))
    address  = str(data.get('address',  'E7E7E7E7E7')).strip().upper()
    datarate = str(data.get('datarate', '2M')).strip()

    # Stop previous radio proc if any.
    if _radio_proc and _radio_proc.proc and _radio_proc.proc.poll() is None:
        _radio_proc.stop()

    # Build the ros2 run command directly so we don't need a launch file.
    import subprocess
    cmd = [
        'ros2', 'run', 'pluto_dashboard', 'crazyflie_bridge',
        '--ros-args',
        '-p', f'radio_channel:={channel}',
        '-p', f'radio_address:={address}',
        '-p', f'radio_datarate:={datarate}',
    ]
    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        # Wrap in _Proc so we can read its log later if needed.
        _radio_proc = _Proc('radio', '', f'CrazyRadio CH{channel}/{address}')
        _radio_proc.proc = proc
        threading.Thread(target=_radio_proc._reader, daemon=True).start()
        node.get_logger().info(f'CrazyRadio bridge started: CH{channel} {address} {datarate}')
        return jsonify({'ok': True, 'uri': f'radio://0/{channel}/{datarate}/{address}'})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500


@app.route('/api/radio/disconnect', methods=['POST'])
def api_radio_disconnect():
    global _radio_proc
    if _radio_proc:
        _radio_proc.stop()
        _radio_proc = None
        node.get_logger().info('CrazyRadio bridge stopped.')
    return jsonify({'ok': True})


@app.route('/api/radio/status')
def api_radio_status():
    if _radio_proc and _radio_proc.proc and _radio_proc.proc.poll() is None:
        return jsonify({'running': True, 'label': _radio_proc.label})
    return jsonify({'running': False})

# PID stub
_pid_state = {'roll': {'p': 40, 'i': 40, 'd': 40},
              'pitch': {'p': 40, 'i': 40, 'd': 40},
              'yaw':   {'p': 85, 'i': 45, 'd': 0}}


@app.route('/api/pid', methods=['GET'])
def api_pid_get():
    return jsonify(_pid_state)


@app.route('/api/pid', methods=['POST'])
def api_pid_set():
    data = request.get_json(silent=True) or {}
    for axis in ('roll', 'pitch', 'yaw'):
        if axis in data:
            for k in ('p', 'i', 'd'):
                if k in data[axis]:
                    _pid_state[axis][k] = int(data[axis][k])
    # Publish to /pluto/set_pid → plutonode forwards via MSP_SET_PID then
    # MSP_EEPROM_WRITE so gains persist across power cycles.
    msg = Int16MultiArray()
    msg.data = [
        _pid_state['roll']['p'],  _pid_state['roll']['i'],  _pid_state['roll']['d'],
        _pid_state['pitch']['p'], _pid_state['pitch']['i'], _pid_state['pitch']['d'],
        _pid_state['yaw']['p'],   _pid_state['yaw']['i'],   _pid_state['yaw']['d'],
    ]
    node.pub_set_pid.publish(msg)
    node.get_logger().info(f'PID → /pluto/set_pid {_pid_state}')
    return jsonify({'ok': True, 'state': _pid_state})


# ── main ─────────────────────────────────────────────────────────────
def main():
    global node
    rclpy.init()
    node = DashboardNode()

    # ROS spin in background; Flask owns main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Find Flask templates next to this file (installed under share/)
    here = Path(__file__).resolve().parent
    app.template_folder = str(here / 'templates')
    if not (here / 'templates' / 'index.html').exists():
        # Fallback to share/ install dir
        try:
            from ament_index_python.packages import get_package_share_directory
            app.template_folder = str(Path(get_package_share_directory('pluto_dashboard')) / 'templates')
        except Exception:
            pass

    print(f'[dashboard] templates: {app.template_folder}')
    print(f'[dashboard] repo root : {REPO_ROOT}')
    try:
        # threaded=True: MJPEG stream is a long-lived generator; without it
        # the single Flask worker blocks /api/telemetry etc. while streaming.
        app.run(host='0.0.0.0', port=5050, debug=False, use_reloader=False, threaded=True)
    finally:
        for p in procs.values():
            try: p.stop()
            except Exception: pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()  #dashboard
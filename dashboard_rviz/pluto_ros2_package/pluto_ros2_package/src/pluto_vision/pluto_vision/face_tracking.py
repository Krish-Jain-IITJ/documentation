"""
Face Tracking Node for Pluto Drone
Subscribes to /plutocamera/image_raw.
Detects faces using OpenCV Haar Cascades.

Calculates X error and sends YAW commands to keep the face centered horizontally.
Calculates Y error and sends THROTTLE commands to keep the face centered vertically.
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
except ImportError as e:
    raise SystemExit(f'face_tracking needs cv_bridge + cv2 installed: {e}')

NEUTRAL = 1500

def _clip(x: float, lo: int = 1000, hi: int = 2000) -> int:
    return max(lo, min(hi, int(round(x))))

class FaceTracker(Node):
    def __init__(self):
        super().__init__('pluto_face_tracker')
        
        self.declare_parameter('enabled', False)
        # Gain for Yaw (rotation) - determines how fast the drone spins to center the face
        self.declare_parameter('gain_yaw', 1.0)
        # Gain for Throttle (up/down) - keep it low for safety, prevents sudden jumps
        self.declare_parameter('gain_throttle', 1.0)
        
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.bridge = CvBridge()
        self.create_subscription(Image, '/plutocamera/image_raw', self._on_image, cam_qos)
        self.pub_cmd = self.create_publisher(PlutoMsg, '/drone_command', 10)

        # Load the default OpenCV Face Detector
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        self.get_logger().info('Face Tracker ready. Run with: --ros-args -p enabled:=true')

    def _on_image(self, msg: Image):
        if not bool(self.get_parameter('enabled').value):
            return
            
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        
        # Detect faces in the image
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))
        
        if len(faces) == 0:
            # No face found, stop rotating/moving vertically, just hover
            self._publish_rc(yaw=NEUTRAL, throttle=NEUTRAL)
            return

        # If multiple faces are found, find the largest one (assume it's the main subject)
        largest_face = max(faces, key=lambda rect: rect[2] * rect[3])
        x, y, w_face, h_face = largest_face
        
        # Calculate the exact center of the face
        cx = x + w_face / 2.0
        cy = y + h_face / 2.0
        
        h, w = bgr.shape[:2]
        
        # Calculate how far the face is from the center of the camera screen
        err_x = cx - w / 2.0  # +ve means face is to the right
        err_y = cy - h / 2.0  # +ve means face is below the center

        gain_yaw = float(self.get_parameter('gain_yaw').value)
        gain_throttle = float(self.get_parameter('gain_throttle').value)
        
        # Rotate Right (Yaw > 1500) if face is on the right
        yaw = _clip(NEUTRAL + err_x * gain_yaw)
        
        # Move Down (Throttle < 1500) if face is below center.
        # Note: image Y axis goes down. So +ve err_y means face is low -> we need to lower the drone.
        throttle = _clip(NEUTRAL - err_y * gain_throttle)
        
        self._publish_rc(yaw=yaw, throttle=throttle)

    def _publish_rc(self, yaw: int, throttle: int):
        msg = PlutoMsg()
        msg.rc_roll     = NEUTRAL  # Do not roll left/right
        msg.rc_pitch    = NEUTRAL  # Do not pitch forward/backward
        msg.rc_yaw      = yaw      # Rotate to track face
        msg.rc_throttle = throttle # Change altitude to track face
        msg.rc_aux1 = NEUTRAL; msg.rc_aux2 = 1000; msg.rc_aux3 = 1000; msg.rc_aux4 = 1500
        msg.pluto_index = 0
        msg.command_type = 0
        msg.is_auto_pilot_on = True 
        self.pub_cmd.publish(msg)

def main():
    rclpy.init()
    node = FaceTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
Do you want it on the Dashboard?
If you want to be able to click a switch on your web dashboard to turn Obstacle Avoidance ON and OFF (just like you do for the camera or the drone itself), we can easily build that!

To do that, we would just need to:

Add a small toggle button to your index.html file.
Add a few lines of code to your dashboard_node.py so that when you click the button, it launches the obstacle_avoidance.py script in the background.
Would you like me to integrate it into your dashboard so you don't have to use the terminal?
Vision-based Obstacle Avoidance Node for Pluto Drone
Subscribes to /plutocamera/image_raw and publishes to /drone_command.
Uses OpenCV to detect large obstacles (via edge density) in the center of the frame.
If an obstacle is detected too close, it sends a 'pitch backward' command.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from custom_msgs.msg import PlutoMsg
import numpy as np

try:
    import cv2
    from cv_bridge import CvBridgenow 
except ImportError as e:
    raise SystemExit(f'obstacle_avoidance needs cv_bridge + cv2 installed: {e}')

NEUTRAL = 1500

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('pluto_obstacle_avoidance')
        
        # Safety switch - Disabled by default
        self.declare_parameter('enabled', False)
        
        # Threshold for how "dense" the edges need to be to trigger avoidance.
        # A higher number means the object must be closer/more textured.
        self.declare_parameter('edge_threshold', 15.0) 
        
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.bridge = CvBridge()
        self.create_subscription(Image, '/plutocamera/image_raw', self._on_image, cam_qos)
        self.pub_cmd = self.create_publisher(PlutoMsg, '/drone_command', 10)

        self.get_logger().info('Obstacle Avoidance ready (enabled=False by default).')
        self.get_logger().info('Run with: --ros-args -p enabled:=true')

    def _on_image(self, msg: Image):
        if not bool(self.get_parameter('enabled').value):
            return
            
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        # Simple Obstacle Detection using Edge Density
        # 1. Convert to grayscale
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        
        # 2. Extract the center region of the image (where the drone is heading)
        h, w = gray.shape
        center_h_start, center_h_end = int(h * 0.3), int(h * 0.7)
        center_w_start, center_w_end = int(w * 0.3), int(w * 0.7)
        
        center_roi = gray[center_h_start:center_h_end, center_w_start:center_w_end]
        
        # 3. Detect edges using Canny
        # Canny edge detection highlights the outlines of objects.
        # As an object gets closer to the camera, it takes up more space, 
        # and its edges become more prominent in the center frame.
        edges = cv2.Canny(center_roi, 100, 200)
        
        # 4. Calculate the density of edges (percentage of white pixels)
        edge_density = (np.sum(edges > 0) / edges.size) * 100.0
        
        threshold = float(self.get_parameter('edge_threshold').value)
        
        if edge_density > threshold:
            self.get_logger().warn(f'OBSTACLE DETECTED! Density: {edge_density:.2f}% - Braking!')
            # Obstacle detected! Pitch backward (less than 1500) to stop/reverse
            self._publish_rc(roll=NEUTRAL, pitch=1350) 
        else:
            # Path clear, hover in place.
            # In a more advanced script, you could let manual control pass through here.
            self._publish_rc(roll=NEUTRAL, pitch=NEUTRAL)

    def _publish_rc(self, roll: int, pitch: int):
        msg = PlutoMsg()
        msg.rc_roll     = roll
        msg.rc_pitch    = pitch
        msg.rc_yaw      = NEUTRAL
        msg.rc_throttle = NEUTRAL  # Assume altitude hold is on
        msg.rc_aux1 = NEUTRAL; msg.rc_aux2 = 1000; msg.rc_aux3 = 1000; msg.rc_aux4 = 1500
        msg.pluto_index = 0
        msg.command_type = 0
        msg.is_auto_pilot_on = True 
        self.pub_cmd.publish(msg)

def main():
    rclpy.init()
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
simple_takeoff.py
─────────────────
Publishes geometry_msgs/Twist on /cmd_vel to simulate takeoff.
Phase 1 (3s): climb straight up at 0.5 m/s
Phase 2 (3s): hover (zero velocity)
Phase 3:      hold hover indefinitely

Run standalone:
  ros2 run drone_sim simple_takeoff
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class SimpleTakeoff(Node):

    def __init__(self):
        super().__init__('simple_takeoff')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._start = time.time()
        # 20 Hz control loop
        self._timer = self.create_timer(0.05, self._tick)
        self.get_logger().info('SimpleTakeoff: starting climb...')

    def _tick(self):
        elapsed = time.time() - self._start
        msg = Twist()

        if elapsed < 3.0:
            # Phase 1 — climb
            msg.linear.z = 0.5   # m/s upward
            self.get_logger().info(
                f'Climbing... z_vel=0.5  elapsed={elapsed:.1f}s', once=False)

        elif elapsed < 6.0:
            # Phase 2 — level off
            msg.linear.z = 0.0
            self.get_logger().info('Hovering.', once=True)

        else:
            # Phase 3 — hold hover
            msg.linear.z = 0.0

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

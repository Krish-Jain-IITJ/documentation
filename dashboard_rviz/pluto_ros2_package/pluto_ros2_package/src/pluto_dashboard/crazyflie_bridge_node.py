"""CrazyRadio PA bridge node for Pluto drone.

Connects to the Pluto drone over a CrazyRadio PA USB dongle instead of WiFi.
The Pluto firmware speaks MSP over the Crazyradio link on channel 80 by default.

Prerequisites (on WSL/Ubuntu):
    pip install cflib
    # Plug in CrazyRadio PA USB dongle BEFORE starting this node.
    # On WSL2: pass through the USB device via usbipd-win.

Usage (standalone):
    ros2 run pluto_dashboard crazyflie_bridge

Usage (via launch with radio enabled):
    ros2 launch pluto_dashboard dashboard_launch.py use_radio:=true radio_channel:=80
"""

import threading
import time

import rclpy
from rclpy.node import Node

from custom_msgs.msg import PlutoMsg


def _cflib_available() -> bool:
    try:
        import cflib  # noqa: F401
        return True
    except ImportError:
        return False


class CrazyflieBridge(Node):
    def __init__(self):
        super().__init__('crazyflie_bridge')

        self.declare_parameter('radio_channel', 80)
        self.declare_parameter('radio_datarate', '2M')
        self.declare_parameter('radio_address', 'E7E7E7E7E7')
        self.declare_parameter('radio_index', 0)

        channel  = self.get_parameter('radio_channel').value
        datarate = self.get_parameter('radio_datarate').value
        address  = self.get_parameter('radio_address').value
        idx      = self.get_parameter('radio_index').value

        self._uri = f'radio://{idx}/{channel}/{datarate}/{address}'
        self.get_logger().info(f'CrazyflieBridge: connecting to {self._uri}')

        if not _cflib_available():
            self.get_logger().error(
                'cflib not installed — run: pip install cflib\n'
                'CrazyRadio bridge will NOT be active.'
            )
            self._cf = None
            return

        try:
            import cflib
            from cflib.crtp import init_drivers
            from cflib.crazyflie import Crazyflie

            init_drivers(enable_debug_driver=False)

            self._cf = Crazyflie(rw_cache='./cache')
            self._cf.connected.add_callback(self._on_connected)
            self._cf.disconnected.add_callback(self._on_disconnected)
            self._cf.connection_failed.add_callback(self._on_conn_failed)

            self._cf.open_link(self._uri)

        except Exception as e:
            self.get_logger().error(f'CrazyRadio init failed: {e}')
            self._cf = None
            return

        # Subscribe to the same /drone_command that WiFi plutonode reads.
        self.create_subscription(
            PlutoMsg, '/drone_command', self._rc_callback, 10
        )

        # Keep-alive: send zero setpoint so the Crazyflie link does not time out.
        self._ka_thread = threading.Thread(target=self._keepalive_loop, daemon=True)
        self._ka_thread.start()

    # ── Crazyflie link callbacks ──────────────────────────────────────

    def _on_connected(self, uri):
        self.get_logger().info(f'CrazyRadio connected: {uri}')

    def _on_disconnected(self, uri):
        self.get_logger().warn(f'CrazyRadio disconnected: {uri}')

    def _on_conn_failed(self, uri, msg):
        self.get_logger().error(f'CrazyRadio connection failed ({uri}): {msg}')

    # ── RC forwarding ─────────────────────────────────────────────────

    def _rc_callback(self, msg: PlutoMsg):
        """Forward dashboard RC (1000–2000) to Crazyflie as normalised setpoint."""
        if self._cf is None:
            return

        def _norm(v):
            """Map [1000, 2000] → [-1.0, +1.0] with 1500 as zero."""
            return (int(v) - 1500) / 500.0

        roll    = _norm(msg.rc_roll)   * 30.0   # degrees
        pitch   = _norm(msg.rc_pitch)  * 30.0   # degrees
        yaw     = _norm(msg.rc_yaw)    * 200.0  # deg/s
        thrust  = max(0.0, (int(msg.rc_throttle) - 1000) / 1000.0)  # 0–1

        # Crazyflie commander: roll (deg), pitch (deg), yawrate (deg/s), thrust (0-65535)
        thrust_raw = int(thrust * 65535)
        try:
            self._cf.commander.send_setpoint(roll, pitch, yaw, thrust_raw)
        except Exception as e:
            self.get_logger().error(f'CrazyRadio send_setpoint failed: {e}')

    def _keepalive_loop(self):
        while rclpy.ok():
            if self._cf is not None:
                try:
                    self._cf.commander.send_setpoint(0, 0, 0, 0)
                except Exception:
                    pass
            time.sleep(0.5)

    def destroy_node(self):
        if self._cf is not None:
            try:
                self._cf.close_link()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int16MultiArray
from sensor_msgs.msg import Imu, BatteryState, Range
from custom_msgs.msg import PlutoMsg
from custom_msgs.msg import PlutoMsgAP
from plutodrone.Communication import *
from plutodrone.Protocol import Protocol
from plutodrone.Common import *
import threading
import time

NONE_COMMAND = 0 
TAKE_OFF = 1 
LAND = 2 

TRIM_MAX = 1000
TRIM_MIN = -1000

MSP_FC_VERSION = 3
MSP_RAW_IMU = 102
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_SET_RAW_RC = 200
MSP_ACC_CALIBRATION = 205
MSP_MAG_CALIBRATION = 206
MSP_SET_MOTOR = 214
MSP_SET_ACC_TRIM = 239
MSP_ACC_TRIM = 240
MSP_EEPROM_WRITE = 250
MSP_SET_POS = 216
MSP_SET_COMMAND = 217

class PlutoNode(Node):

    def __init__(self):
        super().__init__('pluto_node')

        self.is_socket_created = False
        self.is_auto_pilot_on = False
        self.command_type = NONE_COMMAND
        self.command_type_ap = NONE_COMMAND
        self.userRC = [1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000]
        self.userRCAP = [1500, 1500, 1500, 1500]
        self.droneRC = [1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000]

        # Initialize Protocol
        self.pro = Protocol()

        # Subscribe to topics
        self.sub = self.create_subscription(PlutoMsg, '/drone_command', self.read_drone_command, 10)
        self.sub_auto_pilot = self.create_subscription(PlutoMsgAP, '/drone_ap_command', self.read_drone_ap_command, 10)

        # Firmware command topics (MSP passthrough)
        self.create_subscription(Empty, '/pluto/calibrate_acc', lambda _m: self._safe_send(self.pro.sendRequestMSP_ACC_CALIBRATION, 'ACC_CALIB'), 10)
        self.create_subscription(Empty, '/pluto/calibrate_mag', lambda _m: self._safe_send(self.pro.sendRequestMSP_MAG_CALIBRATION, 'MAG_CALIB'), 10)
        self.create_subscription(Empty, '/pluto/eeprom_write',  lambda _m: self._safe_send(self.pro.sendRequestMSP_EEPROM_WRITE,  'EEPROM_WR'), 10)
        self.create_subscription(Int16MultiArray, '/pluto/motor_test', self._on_motor_test, 10)
        self.create_subscription(Int16MultiArray, '/pluto/acc_trim',   self._on_acc_trim,   10)
        self.create_subscription(Int16MultiArray, '/pluto/set_pid',    self._on_set_pid,    10)

        # Telemetry publishers (standard sensor_msgs — rviz / foxglove / rosbag native).
        # High-rate → BEST_EFFORT QoS so subscribers always see the latest sample.
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        tele_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.pub_imu = self.create_publisher(Imu,          '/pluto/imu',      tele_qos)
        self.pub_bat = self.create_publisher(BatteryState, '/pluto/battery',  tele_qos)
        self.pub_alt = self.create_publisher(Range,        '/pluto/altitude', tele_qos)

        self.lock_user_rc = threading.Lock()
        self.lock_command_type = threading.Lock()
        self.lock_command_type_ap = threading.Lock()
        self.lock_send = threading.Lock()  # serialise MSP packet writes across threads

        self.trim_roll = 0
        self.trim_pitch = 0

        self.threads = []

    def _safe_send(self, fn, tag):
        with self.lock_send:
            try:
                fn()
                self.get_logger().info(f'MSP → {tag}')
            except Exception as e:
                self.get_logger().error(f'MSP {tag} failed: {e}')

    def _on_motor_test(self, msg):
        # msg.data = [m0, m1, m2, m3], each 1000..2000. Props OFF.
        motors = [max(1000, min(2000, int(v))) for v in list(msg.data)[:4]]
        with self.lock_send:
            try:
                self.pro.sendRequestMSP_SET_MOTOR(motors)
                self.get_logger().warn(f'MSP → SET_MOTOR {motors}')
            except Exception as e:
                self.get_logger().error(f'SET_MOTOR failed: {e}')

    def _on_set_pid(self, msg):
        """Apply PID gains for roll/pitch/yaw. Expects 9 ints:
        [rP, rI, rD, pP, pI, pD, yP, yI, yD]."""
        if len(msg.data) < 9:
            self.get_logger().warn(f'SET_PID: need 9 values, got {len(msg.data)}')
            return
        d = [int(v) for v in msg.data[:9]]
        pids = {
            'roll':  {'p': d[0], 'i': d[1], 'd': d[2]},
            'pitch': {'p': d[3], 'i': d[4], 'd': d[5]},
            'yaw':   {'p': d[6], 'i': d[7], 'd': d[8]},
        }
        with self.lock_send:
            try:
                self.pro.sendRequestMSP_SET_PID(pids)
                self.pro.sendRequestMSP_EEPROM_WRITE()  # persist across reboot
                self.get_logger().info(f'MSP → SET_PID {pids} + EEPROM')
            except Exception as e:
                self.get_logger().error(f'SET_PID failed: {e}')

    def _on_acc_trim(self, msg):
        # msg.data = [roll_trim, pitch_trim]
        if len(msg.data) < 2:
            return
        r, p = int(msg.data[0]), int(msg.data[1])
        self.trim_roll, self.trim_pitch = r, p
        with self.lock_send:
            try:
                self.pro.sendRequestMSP_SET_ACC_TRIM(r, p)
                self.pro.sendRequestMSP_EEPROM_WRITE()
                self.get_logger().info(f'MSP → SET_ACC_TRIM r={r} p={p} + EEPROM')
            except Exception as e:
                self.get_logger().error(f'SET_ACC_TRIM failed: {e}')

    def read_drone_command(self, msg):
        with self.lock_user_rc:
            self.userRC[0] = msg.rc_roll
            self.userRC[1] = msg.rc_pitch
            self.userRC[2] = msg.rc_throttle
            self.userRC[3] = msg.rc_yaw
            self.userRC[4] = msg.rc_aux1
            self.userRC[5] = msg.rc_aux2
            self.userRC[6] = msg.rc_aux3
            self.userRC[7] = msg.rc_aux4

            if self.command_type == NONE_COMMAND:
                with self.lock_command_type:
                    self.command_type = msg.command_type

            with self.lock_command_type_ap:
                if msg.trim_roll != 0 or msg.trim_pitch != 0:
                    self.trim_roll += msg.trim_roll
                    self.trim_pitch += msg.trim_pitch

                    if self.trim_roll > TRIM_MAX:
                        self.trim_roll = TRIM_MAX
                    elif self.trim_roll < TRIM_MIN:
                        self.trim_roll = TRIM_MIN

                    if self.trim_pitch > TRIM_MAX:
                        self.trim_pitch = TRIM_MAX
                    elif self.trim_pitch < TRIM_MIN:
                        self.trim_pitch = TRIM_MIN

                    self.pro.sendRequestMSP_SET_ACC_TRIM(self.trim_roll, self.trim_pitch)
                    self.pro.sendRequestMSP_EEPROM_WRITE()

    def read_drone_ap_command(self, msg):
        self.get_logger().info('Received Drone AP Command')
        self.get_logger().info('Yaw: %d' % msg.rc_yaw)

        with self.lock_user_rc:
            self.userRCAP[0] = msg.rc_roll
            self.userRCAP[1] = msg.rc_pitch
            self.userRCAP[2] = msg.rc_throttle
            self.userRCAP[3] = msg.rc_yaw

            with self.lock_command_type_ap:
                if self.command_type_ap == NONE_COMMAND:
                    self.command_type_ap = msg.command_type

    def create_socket(self):
        is_socket_created = connectSock()
        self.is_socket_created = is_socket_created

        if is_socket_created:
            self.threads.append(self.create_thread(self.write_function, 2))
            self.threads.append(self.create_thread(self.read_function, 3))
            self.threads.append(self.create_thread(self.service_function, 4))

    def write_function(self, thread_arg):
        requests = [MSP_RC, MSP_ATTITUDE, MSP_RAW_IMU, MSP_ALTITUDE, MSP_ANALOG]
        with self.lock_send:
            self.pro.sendRequestMSP_ACC_TRIM()

        while True:
            with self.lock_user_rc:
                self.droneRC[:] = self.userRC  # Copy the userRC values

            if self.is_auto_pilot_on and self.droneRC[7] == 1500:
                with self.lock_user_rc:
                    self.droneRC[0] += self.userRCAP[0] - 1500
                    self.droneRC[1] += self.userRCAP[1] - 1500
                    self.droneRC[2] += self.userRCAP[2] - 1500
                    self.droneRC[3] += self.userRCAP[3] - 1500

            with self.lock_send:
                self.pro.sendRequestMSP_SET_RAW_RC(self.droneRC)
                self.pro.sendRequestMSP_GET_DEBUG(requests)

                with self.lock_command_type:
                    if self.command_type != NONE_COMMAND:
                        self.pro.sendRequestMSP_SET_COMMAND(self.command_type)
                        self.command_type = NONE_COMMAND
                    elif self.command_type_ap != NONE_COMMAND and self.is_auto_pilot_on and self.droneRC[7] == 1500:
                        with self.lock_command_type_ap:
                            self.pro.sendRequestMSP_SET_COMMAND(self.command_type_ap)
                            self.command_type_ap = NONE_COMMAND

            time.sleep(0.022)  # Sleep for 22 milliseconds

    def read_function(self, thread_arg):
        while True:
          readFrame()

    def service_function(self, thread_arg):
        """Publish telemetry as standard sensor_msgs at ~100 Hz.

        Renamed historically to 'service_function' because it used to call the
        pluto_service to round-trip RC through the dashboard. That path is gone:
        RC now flows purely via the `/drone_command` subscription, and telemetry
        flows purely via these publishers. Dashboard is optional; any ROS node
        can consume these topics directly.
        """
        while True:
            roll, pitch, yaw, battery, rssi, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, alt, rc_aux1, rc_aux2, rc_aux3, rc_aux4, rc_throttle, rc_pitch, rc_yaw, rc_roll = self.pro.returnData()
            current_ma, mah_drawn, mah_remain, soc, auto_land = self.pro.returnBms()

            now = self.get_clock().now().to_msg()
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = 'pluto_imu'
            # Orientation from attitude (decidegrees → rad).
            r = math.radians(roll  / 10.0)
            p = math.radians(pitch / 10.0)
            y = math.radians(yaw)
            cr, sr = math.cos(r/2), math.sin(r/2)
            cp, sp = math.cos(p/2), math.sin(p/2)
            cy, sy = math.cos(y/2), math.sin(y/2)
            imu.orientation.w = cr*cp*cy + sr*sp*sy
            imu.orientation.x = sr*cp*cy - cr*sp*sy
            imu.orientation.y = cr*sp*cy + sr*cp*sy
            imu.orientation.z = cr*cp*sy - sr*sp*cy
            # Linear accel in m/s^2 (Pluto: ~512 counts per g).
            G = 9.80665
            imu.linear_acceleration.x = (acc_x / 512.0) * G
            imu.linear_acceleration.y = (acc_y / 512.0) * G
            imu.linear_acceleration.z = (acc_z / 512.0) * G
            # Angular velocity already deg/s from Protocol (divide-by-8).
            imu.angular_velocity.x = math.radians(gyro_x)
            imu.angular_velocity.y = math.radians(gyro_y)
            imu.angular_velocity.z = math.radians(gyro_z)
            # Covariance unknown.
            imu.orientation_covariance[0] = -1.0
            imu.linear_acceleration_covariance[0] = -1.0
            imu.angular_velocity_covariance[0] = -1.0
            self.pub_imu.publish(imu)

            bat = BatteryState()
            bat.header.stamp = now
            bat.header.frame_id = 'pluto'
            bat.voltage = float(battery)
            bat.current = float(current_ma) / 1000.0     # A (positive = discharge on INA219)
            bat.charge  = float(mah_drawn)  / 1000.0     # Ah drawn so far
            bat.capacity = float(mah_remain) / 1000.0    # Ah still available
            # Prefer firmware SoC; fall back to linear LiPo 3.3–4.2 V.
            bat.percentage = (float(soc) / 100.0) if soc else (
                max(0.0, min(1.0, (float(battery) - 3.3) / 0.9)) if battery else 0.0)
            bat.present = bool(battery)
            bat.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
            # auto_land = 1 means firmware triggered low-voltage land.
            bat.power_supply_health = (BatteryState.POWER_SUPPLY_HEALTH_DEAD
                                       if auto_land else BatteryState.POWER_SUPPLY_HEALTH_GOOD)
            self.pub_bat.publish(bat)

            # Altitude as sensor_msgs/Range (barometer-derived, cm → m).
            rng = Range()
            rng.header.stamp = now
            rng.header.frame_id = 'pluto_baro'
            rng.radiation_type = Range.INFRARED   # no dedicated baro enum; closest available
            rng.field_of_view = 0.0
            rng.min_range = -10.0
            rng.max_range = 1000.0
            rng.range = float(alt) / 100.0
            self.pub_alt.publish(rng)

            time.sleep(0.01)

    def create_thread(self, function, thread_id):
        thread = threading.Thread(target=function, args=(thread_id,))
        thread.start()
        return thread

def main(args=None):
    rclpy.init(args=args)
    node = PlutoNode()
    node.create_socket()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

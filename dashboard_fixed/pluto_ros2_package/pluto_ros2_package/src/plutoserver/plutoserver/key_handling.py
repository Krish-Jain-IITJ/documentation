import rclpy
from rclpy.node import Node
from custom_msgs.msg import PlutoMsg
from std_msgs.msg import Int16

class SendData(Node):
    def __init__(self):
        super().__init__('key_handling')
        self.get_logger().info("Initiliazed node")
        self.command_pub = self.create_publisher(PlutoMsg, '/drone_command', 1)

        self.key_value = 0
        self.cmd = PlutoMsg()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux1 = 1500
        self.cmd.rc_aux2 = 1500
        self.cmd.rc_aux3 = 1500
        self.cmd.rc_aux4 = 1000
        self.cmd.pluto_index = 0
        self.cmd.command_type = 0
        self.cmd.trim_roll = 0
        self.cmd.trim_pitch = 0
        self.cmd.is_auto_pilot_on = False

        self.create_subscription(Int16, '/input_key', self.identify_key, 1)

    def arm(self):
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1500
        self.cmd.is_auto_pilot_on = False
        self.command_pub.publish(self.cmd)
        # self.node.sleep(1)

    def box_arm(self):
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 1500
        self.cmd.is_auto_pilot_on = False
        self.command_pub.publish(self.cmd)
        # self.node.sleep(0.5)

    def disarm(self):
        self.cmd.rc_throttle = 1300
        self.cmd.rc_aux4 = 1200
        self.command_pub.publish(self.cmd)
        # self.node.sleep(0.5)

    def identify_key(self, msg):
        self.key_value = msg.data
        # self.get_logger().info("MSG=%d" % msg.data)
        if self.key_value == 70:
            if self.cmd.rc_aux4 == 1500:
                self.disarm()
            else:
                self.arm()
        elif self.key_value == 10:
            self.forward()
            
        elif self.key_value == 30:
            self.left()
        elif self.key_value == 40:
            self.right()
        elif self.key_value == 80:
            self.reset()
        elif self.key_value == 90:
            self.cmd.is_auto_pilot_on = not self.cmd.is_auto_pilot_on
        elif self.key_value == 50:
            self.increase_height()
        elif self.key_value == 60:
            self.decrease_height()
        elif self.key_value == 110:
            self.backward()
        elif self.key_value == 130:
            self.take_off()
        elif self.key_value == 140:
            self.land()
        elif self.key_value == 150:
            self.left_yaw()
        elif self.key_value == 160:
            self.right_yaw()
        # self.command_pub.publish(self.cmd)

    def forward(self):
        self.cmd.rc_pitch = 1600
        self.command_pub.publish(self.cmd)

    def backward(self):
        self.cmd.rc_pitch = 1400
        self.command_pub.publish(self.cmd)

    def left(self):
        self.cmd.rc_roll = 1400
        self.command_pub.publish(self.cmd)

    def right(self):
        self.cmd.rc_roll = 1600
        self.command_pub.publish(self.cmd)

    def left_yaw(self):
        self.cmd.rc_yaw = 1200
        self.command_pub.publish(self.cmd)

    def right_yaw(self):
        self.cmd.rc_yaw = 1800
        self.command_pub.publish(self.cmd)

    def reset(self):
        self.cmd.rc_roll = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.command_type = 0
        self.command_pub.publish(self.cmd)

    def increase_height(self):
        self.cmd.rc_throttle = 1800
        self.command_pub.publish(self.cmd)

    def decrease_height(self):
        self.cmd.rc_throttle = 1300
        self.command_pub.publish(self.cmd)

    def take_off(self):
        self.disarm()
        self.box_arm()
        self.cmd.command_type = 1
        self.command_pub.publish(self.cmd)

    def land(self):
        self.cmd.command_type = 2
        self.command_pub.publish(self.cmd)

def main(args=None):
    rclpy.init()
    test = SendData()
    rclpy.spin(test)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

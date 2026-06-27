import rclpy
from rclpy.node import Node
from custom_msgs.srv import PlutoPilot
from std_msgs.msg import Float32

class RequestData(Node):
    def __init__(self):
        super().__init__('pluto_receive_data')
        self.node = rclpy.create_node('drone_board_data')
        self.srv = self.node.create_service(PlutoPilot, 'pluto_service', self.access_data)

    def access_data(self, request, response):
        print("receiving")
        print("accx =", request.acc_x, "accy =", request.acc_y, "accz =", request.acc_z)
        print("gyrox =", request.gyro_x, "gyroy =", request.gyro_y, "gyroz =", request.gyro_z)
        print("magx =", request.mag_x, "magy =", request.mag_y, "magz =", request.mag_z)
        print("roll =", request.roll, "pitch =", request.pitch, "yaw =", request.yaw)
        print("altitude =", request.alt)
        print("battery =", request.battery, "Power Consumed =", request.rssi)
        self.node.get_logger().info('Sending response...')
        response.rc_aux2 = 1500
        return response
    
def main(args=None):
    rclpy.init()
    test = RequestData()
    rclpy.spin(test)
    rclpy.shutdown()
if __name__ == '__main__':
    main()

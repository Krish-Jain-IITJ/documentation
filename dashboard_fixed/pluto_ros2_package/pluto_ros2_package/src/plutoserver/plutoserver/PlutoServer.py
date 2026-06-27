import rclpy
from rclpy.node import Node
from custom_msgs.srv import PlutoPilot

class PlutoServer(Node):
    def __init__(self):
        super().__init__('pluto_service')
        self.srv = self.create_service(PlutoPilot, 'pluto_service', self.handle_request)
        self.get_logger().info('Ready to Provide Pluto Service')

    def handle_request(self, request, response):
        # Handle the service request here
        self.get_logger().info('Received Service Request:')
        self.get_logger().info('Ax=%f, Ay=%f, Az=%f' % (request.acc_x, request.acc_y, request.acc_z))
        self.get_logger().info('Gx=%f, Gy=%f, Gz=%f' % (request.gyro_x, request.gyro_y, request.gyro_z))
        self.get_logger().info('Mx=%f, My=%f, Mz=%f' % (request.mag_x, request.mag_y, request.mag_z))
        self.get_logger().info('Roll=%i, Pitch=%i, Yaw=%i' % (request.roll, request.pitch, request.yaw))
        self.get_logger().info('Altitude=%f' % request.alt)
        self.get_logger().info('Battery=%f RSSI=%i' % (request.battery, request.rssi))

        # Populate the response fields
        response.rc_aux1 = 1800

        self.get_logger().info('Sending back response: [%ld]'% response.rc_aux1)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = PlutoServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

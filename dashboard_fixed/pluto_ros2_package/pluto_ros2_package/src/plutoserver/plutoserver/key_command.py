import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from pynput import keyboard

msg = """
Control Your Drone!
---------------------------
Moving around:
   u    i    o
   j    k    l
   n    m    ,


spacebar : arm or disarm
w : increase height
s : decrease height
q : take off
e : land
a : yaw left
d : yaw right
t : auto pilot on/off
Up arrow : go forward
Down arrow : go backward
Left arrow : go left
Right arrow : go right

CTRL+C to quit
"""

class KeyCommand(Node):

    def __init__(self):
        super().__init__('key_command')
        self.pub = self.create_publisher(Int16, '/input_key', 1)
        self.rate = self.create_rate(100)
        self.msg_pub = 0
        self.keyboard_control = {
            'Key.up': 10,
            'Key.left': 30,
            'Key.right': 40,
            'w': 50,
            's': 60,
            'Key.space': 70,
            'r': 80,
            't': 90,
            'p': 100,
            'Key.down': 110,
            'n': 120,
            'q': 130,
            'e': 140,
            'a': 150,
            'd': 160,
            '+': 15,
            '1': 25,
            '2': 30,
            '3': 35,
            '4': 45
        }

    def on_press(self, key):
        
        try:
            char = key.char
            self.handle_key(char)
        except AttributeError:
            # self.get_logger().info(f"Pressed key: {key}")
            self.handle_key(str(key))

    def handle_key(self, key):
        if key == 'Key.esc':
            self.get_logger().info("Exiting...")
            rclpy.shutdown()
        elif key in self.keyboard_control:
            self.msg_pub = self.keyboard_control[key]
            # self.get_logger().info(f"Message: {self.msg_pub}")
            self.pub.publish(Int16(data=self.msg_pub))
        else:
            self.msg_pub = 80
            self.pub.publish(Int16(data=self.msg_pub))

    def getKey(self):
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()

    def run(self):
        self.get_logger().info("Running...")
        while rclpy.ok():
            self.getKey()  # get the key from keyboard

        self.get_logger().info("KeyCommand node finished.")


def main(args=None):
    rclpy.init(args=args)
    key_command_node = KeyCommand()
    key_command_node.run()
    
if __name__ == '__main__':
    main()

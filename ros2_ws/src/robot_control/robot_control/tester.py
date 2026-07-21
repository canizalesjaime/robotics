import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Tester(Node):
    def __init__(self):
        super().__init__('tester_node')
        self.cmd_sub = self.create_subscription(Float32, 'cmd_motor', self.callback, 10)
 

    def callback(self, msg):
        print(msg.data)


def main():
    try:
        rclpy.init()
        node = Tester()
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
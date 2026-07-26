import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool
import time

class Tester(Node):
    def __init__(self):
        super().__init__('tester_node')
        #self.cmd_sub = self.create_subscription(Float32, 'li_distance', self.callback, 10)

        self.cmd_pub = self.create_publisher(String,'cmd_motor',10)
        #self.base_pub = self.create_publisher(Bool,'cmd_base', 10)
        self.timer = self.create_timer(1.0, self.callback)
 

    def callback(self):
        #print(msg.data)

        s_msg=String()
        s_msg.data="f"
        self.cmd_pub.publish(s_msg)
        time.sleep(5)
        s_msg.data="b"
        self.cmd_pub.publish(s_msg)
        time.sleep(5)
        s_msg.data="s"
        self.cmd_pub.publish(s_msg)
        time.sleep(5)

        # b_msg = Bool()
        # b_msg.data=True
        # self.base_pub.publish(b_msg)
        #time.sleep(10)
        #b_msg.data=False
        #base_pub.publish(b_msg)


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
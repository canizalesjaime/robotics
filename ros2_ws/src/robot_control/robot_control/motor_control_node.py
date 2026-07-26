import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

import os
import sys
sys.path.append(
    os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..", "..", "..", "..",
            "backend"
        )
    )
)
from motor_node import MotorNode


class MotorRos(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.cmd_sub = self.create_subscription(String, 'cmd_motor', self.cmd_callback, 10)
        self.speed_sub = self.create_subscription(Int32, 'cmd_speed', self.speed_callback, 10)
        self.motors=MotorNode()

    def cmd_callback(self, msg):
        self.motors.move(msg.data)
    
    def speed_callback(self,msg):
        self.motors.set_speed(msg.data)
       

def main():
    rclpy.init()
    node = MotorRos()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
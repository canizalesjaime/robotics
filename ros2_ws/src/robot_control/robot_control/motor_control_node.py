import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import curses
import gpiod
import lgpio as GPIO
from .....backend.motor_node import MotorNode


class MotorRos(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.cmd_sub = self.create_subscription(String, 'cmd_motor', self.cmd_callback, 10)
        self.motors=MotorNode()

        
    def cmd_callback(self, msg):
        self.motors.move(msg.data)
       

def main():
    rclpy.init()
    node = MotorRos()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

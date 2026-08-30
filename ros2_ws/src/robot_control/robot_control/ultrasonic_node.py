import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import robot_control.paths
from ultrasonic import Ultrasonic

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.publisher_ = self.create_publisher(Float32, 'distance', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.sensor = Ultrasonic()


    def timer_callback(self):
        dist = self.sensor.get_distance()
        msg = Float32()
        msg.data = dist
        self.publisher_.publish(msg)

   
def main():
    try:
        rclpy.init()
        node = UltrasonicNode()
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
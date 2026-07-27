import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import robot_control.paths
from lunar import LunarNode


class LunarRos(Node):
    def __init__(self):
        super().__init__('lunar_node')
        self.publisher_ = self.create_publisher(Float32, 'distance', 10)
        self.timer = self.create_timer(.5, self.timer_callback)
        self.sensor = LunarNode()

    def timer_callback(self):
        d,s = self.sensor.get_distance()
        msg = Float32()
        msg.data = float(d)
        print(msg.data)
        self.publisher_.publish(msg)


def main():
    try:
        rclpy.init()
        node = LunarRos()
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

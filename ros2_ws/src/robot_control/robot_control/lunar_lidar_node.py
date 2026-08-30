import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
#from rclpy.qos import qos_profile_sensor_data,QoSProfile, ReliabilityPolicy

import robot_control.paths
from lunar_lidar import LunarNode
import time



class LunarRos(Node):
    def __init__(self):
        super().__init__('lunar_node')
        #self.sensor_qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher_ = self.create_publisher(Float32, 'distance', 1)
        self.timer = self.create_timer(.1, self.timer_callback)
        self.sensor = LunarNode()

    def timer_callback(self):
        d,s = self.sensor.get_distance()
        msg = Float32()
        msg.data = float(d)
        self.get_logger().info(f"distance: {d} meters, strength: {s} {time.time()}")
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

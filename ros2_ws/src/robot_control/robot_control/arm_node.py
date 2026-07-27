#ros2 topic pub /servo_angles std_msgs/Int32MultiArray "data: [90, 90, 90, 90]"
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState


import robot_control.paths
from arm import ArmNode 

class ArmRos(Node):
    def __init__(self):
        super().__init__('arm_ros')
        self.angle_sub = self.create_subscription(JointState,'cmd_arm',
                                                  self.angle_callback, 10)
        self.base_sub = self.create_subscription(Bool,'cmd_base',
                                                 self.base_callback, 10)
        self.arm = ArmNode()
        self.rotating = False
        self.rotate_timer = self.create_timer(0.1, self.rotate_callback)
          
    def angle_callback(self, msg):
        angles= [math.degrees(i) for i in msg.position]
        self.arm.set_angles_api(angles)

    def base_callback(self, msg):
        self.rotating=msg.data

    def rotate_callback(self):
            if self.rotating:
                self.arm.move_smooth("base",150)
                self.arm.move_smooth("base",30)
        

    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info('Servo Controller Node stopped and GPIO released.')

def main(args=None):
    rclpy.init(args=args)
    node = ArmRos()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

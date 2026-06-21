#ros2 topic pub /servo_angles std_msgs/Int32MultiArray "data: [90, 90, 90, 90]"
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Int32
from .....backend.arm import ArmNode 

class ArmRos(Node):
    def __init__(self):
        super().__init__('arm_ros')
        self.angle_sub = self.create_subscription(JointState,'cmd_arm',
                                                  self.angle_callback, 10)
        self.base_sub = self.create_subscription(Int32,'cmd_base',
                                                 self.base_callback, 10)
        self.arm = ArmNode()
          
    def angle_callback(self, msg):
        angles= [math.degrees(i) for i in msg.position]
        self.arm.set_angles_api(angles)

    def angle_callback(self, msg):
       self.arm.rotate_loop()

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

# if __name__ == '__main__':
#     main()
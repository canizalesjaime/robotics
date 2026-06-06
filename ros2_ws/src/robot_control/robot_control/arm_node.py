#ros2 topic pub /servo_angles std_msgs/Int32MultiArray "data: [90, 90, 90, 90]"

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from .....backend.arm import ArmNode 

class ArmRos(Node):
    def __init__(self):
        super().__init__('arm_ros')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'servo_angles',  # Topic name
            self.listener_callback,
            10
        )
        self.arm = ArmNode()
        
   
    def listener_callback(self, msg):
        self.arm.set_angles_api(msg)

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
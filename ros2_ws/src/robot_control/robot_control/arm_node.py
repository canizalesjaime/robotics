#ros2 topic pub /servo_angles std_msgs/Int32MultiArray "data: [90, 90, 90, 90]"
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import JointState, LaserScan


import robot_control.paths
from arm import ArmNode 

class ArmRos(Node):
    def __init__(self):
        super().__init__('arm_ros')
        # self.angle_sub = self.create_subscription(JointState,'cmd_arm',
        #                                           self.angle_callback, 10)
        self.base_sub = self.create_subscription(Bool,'cmd_base',
                                                 self.base_callback, 10)
        self.create_subscription(Float32,'distance',self.distance_callback,1)

        self.joint_pub = self.create_publisher(JointState,"/joint_states",10)
        self.scan_pub = self.create_publisher(LaserScan,"/scan",10)

        self.arm = ArmNode()
        self.rotating = True
        self.lidar_data=-1
        self.rotate_timer = self.create_timer(0.1, self.rotate_callback)
          
    # def angle_callback(self, msg):
    #     angles= [math.degrees(i) for i in msg.position]
    #     self.arm.set_angles_api(angles)

    def base_callback(self, msg):
        self.rotating=msg.data

    def distance_callback(self, msg):
        self.lidar_data = msg.data  

    def rotate_callback(self):
            if self.rotating:
                msg = JointState()
                msg.name = ["servo_rotation_joint"]

                scan = LaserScan()
                scan.header.stamp = self.get_clock().now().to_msg()
                scan.header.frame_id = "lidar_link"
                scan.angle_min = math.radians(30)
                scan.angle_max = math.radians(150)
                scan.angle_increment = math.radians(10)
                scan.ranges=[]
                scan.time_increment = 0.05
                scan.scan_time = 0.6
                scan.range_min = 0.04
                scan.range_max = 4.0

                for angle in range(30,160,10):
                    self.arm.move_smooth("base",angle)
                    angle = math.radians(angle)
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.position = [angle]
                    scan.ranges.append(self.lidar_data)
                    self.joint_pub.publish(msg)


                scan.intensities = []
                print(msg)
                print(scan)
                self.scan_pub.publish(scan)
                self.arm.move_smooth("base",30)#reset
                      

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

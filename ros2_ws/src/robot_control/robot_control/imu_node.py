#sudo apt install ros-${ROS_DISTRO}-tf-transformations
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature

import robot_control.paths
from imu_sensor import ImuSensor

class ImuSensorNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(1.0, self.publish_sensor_data)
        self.get_logger().info("BNO055 node started.")
        self.sensor = ImuSensor()

    
    def publish_sensor_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        qw, qx, qy, qz=self.sensor.get_quaternion()
        imu_msg.orientation.w = qw 
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        
        ax,ay,az=self.sensor.get_acceleration()
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        rx,ry,rz = self.sensor.get_gyro()
        imu_msg.angular_velocity.x = rx  
        imu_msg.angular_velocity.y = ry
        imu_msg.angular_velocity.z = rz

        # Optionally set covariance if known (identity = unknown)
        imu_msg.linear_acceleration_covariance[0] = -1.0
        imu_msg.angular_velocity_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)



def main(args=None):
    rclpy.init(args=args)
    node = ImuSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

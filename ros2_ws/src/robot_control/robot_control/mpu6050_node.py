#sudo apt install ros-${ROS_DISTRO}-tf-transformations
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature
from tf_transformations import quaternion_from_euler
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

from mpu6050_node_b import Mpu6050Node

class Mpu6050Ros(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.temp_pub = self.create_publisher(Temperature, 'imu/temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_sensor_data)
        self.get_logger().info("MPU6050 node started.")
        self.sensor = Mpu6050Node()

    
    def publish_sensor_data(self):
        data=self.sensor.sensor_data()

        # Populate IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        yaw=0
        qx, qy, qz, qw = quaternion_from_euler(data["roll"],data["pitch"],yaw)

        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw 

        imu_msg.linear_acceleration.x = data["lin_accel.x"]
        imu_msg.linear_acceleration.y = data["lin_accel.y"]
        imu_msg.linear_acceleration.z = data["lin_accel.z"]

        imu_msg.angular_velocity.x = data["ang_vel.x"]  
        imu_msg.angular_velocity.y = data["ang_vel.y"]
        imu_msg.angular_velocity.z = data["ang_vel.z"]

        # Optionally set covariance if known (identity = unknown)
        imu_msg.linear_acceleration_covariance[0] = -1.0
        imu_msg.angular_velocity_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)

        print(data)
        # Publish temperature
        temp_msg = Temperature()
        temp_msg.header = imu_msg.header
        temp_msg.temperature = data["temp"]
        self.temp_pub.publish(temp_msg)



def main(args=None):
    rclpy.init(args=args)
    node = Mpu6050Ros()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

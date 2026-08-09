import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import JointState

import robot_control.paths
from motor_node import MotorNode


class MotorRos(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.cmd_sub = self.create_subscription(String, 'cmd_motor', self.cmd_callback, 10)
        self.speed_sub = self.create_subscription(Int32, 'cmd_speed', self.speed_callback, 10)
        self.joint_pub = self.create_publisher(JointState,"/wheel_states",10)

        self.timer = self.create_timer(0.05,self.publish_joint_states)

        self.motors=MotorNode()

        # Example conversion factor:Replace with your actual encoder 
        # calculation later. Get Wheel radius, 12 pulses per revolution
        self.TICK_TO_RAD = 0.01

    def cmd_callback(self, msg):
        self.motors.move(msg.data)
    
    def speed_callback(self,msg):
        self.motors.set_speed(msg.data)

    def publish_joint_states(self):
        left_ticks, right_ticks = self.motors.get_ticks()
        left_vel, right_vel = self.motors.get_wheel_velocities()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["left_wheel_joint","right_wheel_joint"]

        msg.position = [left_ticks * self.TICK_TO_RAD, right_ticks * self.TICK_TO_RAD]

        msg.velocity = [left_vel,right_vel]

        self.joint_pub.publish(msg)
        

def main():
    rclpy.init()
    node = MotorRos()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#convert this to ros2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

x = 0
y = 0
yaw = 0

###############################################################################


def poseCallback(pose_message):
    global x, y, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta

###############################################################################


def move(speed, distance, is_forward):
    vel_msg = Twist()
    x0 = x
    y0 = y

    if is_forward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rclpy.Rate(10)

    while distance_moved < distance:
        vel_pub.publish(vel_msg)
        loop_rate.sleep()
        distance_moved = abs(math.sqrt(((x-x0)**2) + ((y-y0)**2)))
        print(distance_moved, x, y)

    vel_msg.linear.x = 0
    vel_pub.publish(vel_msg)

###############################################################################


def rotate(angular_speed_degree, relative_angle_degree, clockwise):
    vel_msg = Twist()
    angular_speed = math.radians(abs(angular_speed_degree))

    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    loop_rate = rclpy.Rate(10)
    current_angle_degree = 0.0
    t0 = rclpy.Time.now().to_sec()

    while current_angle_degree < relative_angle_degree:
        vel_pub.publish(vel_msg)
        loop_rate.sleep()
        t1 = rclpy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree

    vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)

###############################################################################


def setDesiredOrientation(desired_angle_radians):
    relative_angle_radians = desired_angle_radians - yaw
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    rotate(20, math.degrees(abs(relative_angle_radians)), clockwise)

###############################################################################


def goToGoal(x_goal, y_goal):
    setDesiredOrientation(0)
    setDesiredOrientation(math.atan2(y_goal-y, x_goal-x))
    move(1, abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2))), True)

###############################################################################


def smoothMove(x_goal, y_goal):
    distance = 1
    vel_msg = Twist()
    # for smooth optimize scalar for distance and desired_angle_goal
    while distance > .01:
        # the speed is always less than the remaining distance to be covered
        distance = .5*abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))
        # note this is angular velocity.
        desired_angle_goal = 4*(math.atan2(y_goal-y, x_goal-x) - yaw)
        vel_msg.linear.x = distance
        vel_msg.angular.z = desired_angle_goal
        vel_pub.publish(vel_msg)

###############################################################################
def gridClean():
    setDesiredOrientation(0)
    alternate, start, end = 1, 1, 10
    for i in range(start, end):
        if alternate > 0:
            goToGoal(end-i, start)
            goToGoal(end-i, end-1)
        else:
            goToGoal(end-i, end-1)
            goToGoal(end-i, start)
        alternate = alternate*-1

###############################################################################
def spiralClean():
    loop_rate = rclpy.Rate(1)
    i = 0

    while((x < 10) and (y < 10)):
        i = i + 1
        vel_pub.publish(Twist(linear=Vector3(i, 0, 0),
                              angular=Vector3(0, 0, 4)))
        loop_rate.sleep()

###############################################################################
if __name__ == '__main__':
    try:
        rclpy.init_node("turtlesim_motion_pose", anonymous=True)
        vel_pub = rclpy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        pose_sub = rclpy.Subscriber('/turtle1/pose', Pose, poseCallback)
        time.sleep(2)
        gridClean()

    except rclpy.ROSInterruptException:
        rclpy.loginfo("node terminated")

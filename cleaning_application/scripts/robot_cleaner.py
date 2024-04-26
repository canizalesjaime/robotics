#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

x=0
y=0
yaw=0

def poseCallback(pose_message):
    global x, y, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta

def move( speed, distance, is_forward ):
    vel_msg = Twist()        
    global x, y
    x0 = x
    y0 = y
    
    if is_forward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    while distance_moved < distance:
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
        distance_moved = abs ( math.sqrt(((x-x0)**2) + ((y-y0)**2)) )
        print(distance_moved)

    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

def rotate( angular_speed_degree, relative_angle_degree, clockwise ):
    vel_msg = Twist()
    angular_speed=math.radians(abs(angular_speed_degree))

    if clockwise:
        vel_msg.angular.z = -abs( angular_speed )
    else:
        vel_msg.angular.z = abs ( angular_speed )

    loop_rate = rospy.Rate(10)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    current_angle_degree = 0.0
    t0 = rospy.Time.now().to_sec()
    
    while current_angle_degree < relative_angle_degree:
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def setDesiredOrientation(desired_angle_radians):
    relative_angle_radians = desired_angle_radians - yaw
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    rotate( 30, math.degrees(abs(relative_angle_radians)), clockwise)

def goToGoal(x_goal, y_goal):
    global x,y,yaw
    vel_msg = Twist()
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    distance = 1
    while distance > .01:
        distance = .5*abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))
        desired_angle_goal = 4*(math.atan2(y_goal-y, x_goal-x) - yaw )
        vel_msg.linear.x = distance
        vel_msg.angular.z = desired_angle_goal
        velocity_publisher.publish(vel_msg)

def gridClean():
    desired_pose = Pose()
    desired_pose.x = 1
    desired_pose.y = 1
    desired_pose.theta = 0

    goToGoal(1.0,1.0)

    setDesiredOrientation(0)

    move(2.0, 9.0, True)
    rotate(abs(math.radians(20)),abs(math.radians(90)) , False)
    move(2.0, 9.0, True)
    rotate(abs(math.radians(20)), abs(math.radians(90)), False)
    move(2.0, 1.0, True)
    rotate(abs(math.radians(20)), abs(math.radians(90)), False)
    move(2.0, 9.0, True)
    rotate(abs(math.radians(20)), abs(math.radians(90)), True)
    move(2.0, 1.0, True)
    rotate(abs(math.radians(20)), abs(math.radians(90)), True)
    move(2.0, 9.0, True)
    #pass

def spiralClean():
    vel_msg = Twist()
    loop_rate = rospy.Rate(1)
    wk = 4
    rk = 0

    while((x<10.5) and (y<10.5)):
        rk=rk+1
        vel_msg.linear.x =rk
        vel_msg.linear.y =0
        vel_msg.linear.z =0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z =wk
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)



if __name__ == '__main__':
    try:
        rospy.init_node("turtlesim_motion_pose", anonymous=True)
        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, poseCallback)
        time.sleep(2)
       # move(1.0, 2.0, True)
       # rotate(30, 90, True)
       # setDesiredOrientation(math.radians(180))
      #  goToGoal(9.0,9.0)
        gridClean()
        #spiralClean()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")



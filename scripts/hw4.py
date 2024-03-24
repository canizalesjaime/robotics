import turtle
import math
import time
import numpy as np 

x = 0
y = 0
yaw = 0

###############################################################################
class Vector3():
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y=y
        self.z=z

###############################################################################
class Twist:
    def __init__(self, linear=Vector3(), angular=Vector3()):
        self.linear=linear
        self.angular=angular

###############################################################################
def pose_callback(pose_message):
    global x, y, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta
    t=np.array([ [math.cos(yaw),-math.sin(yaw),x],
                 [math.sin(yaw),math.cos(yaw),y],
                 [0,0,1] ])
    a_p1=np.array([2,1,1])
    w_p1=t@a_p1

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

    while distance_moved < distance:
        #vel_pub.publish(vel_msg)
        time.sleep(1)
        distance_moved = abs(math.sqrt(((x-x0)**2) + ((y-y0)**2)))

###############################################################################
def rotate(angular_speed_degree, relative_angle_degree, clockwise=False):
    t0=time.time()
    vel_msg = Twist()
    angular_speed = math.radians(abs(angular_speed_degree))

    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    current_angle_degree = 0.0

    while current_angle_degree < relative_angle_degree:
        #vel_pub.publish(vel_msg)
        t1 = time.time()
        current_angle_degree = (t1-t0)*angular_speed_degree

###############################################################################
def set_desired_orientation(desired_angle_radians):
    relative_angle_radians = desired_angle_radians - yaw
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    rotate(20, math.degrees(abs(relative_angle_radians)), clockwise)

###############################################################################
def go_to_goal(x_goal, y_goal):
    set_desired_orientation(0)
    set_desired_orientation(math.atan2(y_goal-y, x_goal-x))
    move(1, abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2))), True)

###############################################################################
def smooth_move(x_goal, y_goal):
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
        #vel_pub.publish(vel_msg)

###############################################################################
def grid_clean():
    set_desired_orientation(0)
    alternate, start, end = 1, 1, 11
    for i in range(start, end):
        if alternate > 0:
            go_to_goal(end-i, start)
            go_to_goal(end-i, end-1)
        else:
            go_to_goal(end-i, end-1)
            go_to_goal(end-i, start)
        alternate = alternate*-1

###############################################################################
def spiral_clean():
    i = 0
    while((x < 10) and (y < 10)):
        i = i + 1
        #vel_pub.publish(Twist(linear=Vector3(i, 0, 0),
        #                      angular=Vector3(0, 0, 4)))
        time.sleep(1)


###############################################################################
if __name__ == '__main__':
    ts = turtle.getscreen()
    turtle.showturtle()
    turtle.position()
    turtle.forward(15)
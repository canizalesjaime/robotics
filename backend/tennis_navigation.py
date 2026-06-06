# 
# Title          : tennis_navigation.py
# Author         : 
# Created on     :    
# Description    : Detects a tennis ball based on it yellowness using opencv      
#                  and computes its coordinates in the camera depth frame. 
#                  Creates a new frame for the tennis ball with  
#                  head_camera_depth_frame being its parent frame.      
# Usage          : rosrun apollo_detection_systems tennis_navigation.py
# Modifications  : 
# Notes          : 1. convert to ros2 
#                  2. run on home_bot(make a map with ultrasonic)   
# 
###############################################################################
import cv2
import numpy as np
import time

import rospy
import message_filters
import tf
import tf2_ros
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray

from intrinsic_parameters import get_intrinsic_params
from navigation_client import NavigationClient
from transformation_client import TransformClient 
from visualize import show_pose_arr

# Globals: ####################################################################
bridge = CvBridge()
cv_rgb = None # Holds messages of type Image
cv_depth = None # Holds messages of type Image
get_next_point = False 
yellowLower =(30, 150, 100)
yellowUpper = (50, 255, 255)
transform_client = TransformClient()
navigation_client = NavigationClient()


def callback(ros_rgb, ros_depth):
    global get_next_point, bridge, cv_rgb, cv_depth
    try:
        cv_rgb = bridge.imgmsg_to_cv2(ros_rgb, "bgr8")
        cv_depth = bridge.imgmsg_to_cv2(ros_depth)
    except CvBridgeError as e:
        print(e)  
    get_next_point = True


def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv image",hsv_image)
    #cv2.waitKey(0)
     
    #define a mask using the lower and upper bounds of the yellow color 
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)
    return mask


def getContours(binary_image):      
    _,contours, hierarchy = cv2.findContours(binary_image, 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)
    return contours


def draw_ball_contour(binary_image, rgb_image, contours, img_pub):
    black_image = np.zeros([binary_image.shape[0], 
                           binary_image.shape[1],3],'uint8')
    
    for c in contours:
        area = cv2.contourArea(c)
        perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if (area>100):
            cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
            cx, cy = get_contour_center(c)
            cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),5,(150,150,255),-1)
            #print ("Area: {}, Perimeter: {}".format(area, perimeter))
    #print ("number of contours: {}".format(len(contours)))
    #cv2.imshow("RGB Image Contours",rgb_image)
    #cv2.waitKey(0)
    #cv2.imshow("Black Image Contours",black_image)
    #cv2.waitKey(1)
    ros_tennis_img = bridge.cv2_to_imgmsg(rgb_image, "bgr8")
    img_pub.publish(ros_tennis_img)


###############################################################################
#   Auxillary function to draw_ball_contour, finds center of contours.
def get_contour_center(contour):
    # what does this do ?
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy


###############################################################################
#   Parameters:
#       contours     - contours for the image 
#
#   Purpose:  Encloses each contour found in the image, to a circle, if the 
#             circles area is greater than 100, find the center point of the 
#             the circle in the depth image. Finds the average x coordinate,
#             and y-coordinate by considering a neighborhood of size 100 
#             w.r.t. the center point of the circle in the depth image. The 
#             x and y coordinates computed, are the coordinates of the tennis
#             ball w.r.t. the head_camera_depth_frame, create a frame for 
#             the tennis ball setting the head_camera_depth_frame as its parent.
#
#   Return:   None
def transform_tennis_coordinates(contours, pose_pub):
    #randomly select a contour to send as a location          
    for c in contours:
        p_X = 0.0 # x of point p in camera frame
        p_Y = 0.0 # y of point p 
        p_Z = 0.0 # z of point p
        divider = 0
        distance = 0
        area = cv2.contourArea(c)
        perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        int_x = int(x)
        int_y = int(y)
        #cx, cy = get_contour_center(c)
        intrinsic_arr = get_intrinsic_params()
        for i in range( int_x - 10, int_x + 10 ):
            for j in range( int_y - 10, int_y + 10 ):
                if  (i < 640 and j < 480) and (cv_depth[j][i]==cv_depth[j][i]):
                    distance = cv_depth[j][i]
                    divider += 1
                    p_X += distance 
                    p_Y += -(x - intrinsic_arr[0])*distance/intrinsic_arr[2]
                    #p_Z += -(y - intrinsic_arr[1])*distance/intrinsic_arr[3]

        #rate = rospy.Rate(10.0)
        if divider > 0 and area > 100:
            p_X = p_X/divider
            p_Y = p_Y/divider
            print("camera_frame:",p_X,p_Y)
            global navigation_client, transform_client
            map_frame_pt=transform_client.transform_pt("head_camera_depth_frame", 'map', [p_X,p_Y,1])
            print("map_frame:",map_frame_pt[0],map_frame_pt[1])
            show_pose_arr(0,'map',map_frame_pt, 0, pose_pub)
            navigation_client.move_to_goal(map_frame_pt[0],map_frame_pt[1],Quaternion(0, 0, 0, 1))
            break


def main():
    # Subscribers:
    depth_sub = message_filters.Subscriber("/head_camera/depth/image", Image,
                                            queue_size=1)#, buff_size=2**28)
    rgb_sub = message_filters.Subscriber("/head_camera/rgb/image_raw", Image, 
                                          queue_size=1)# buff_size=2**28)
    depth_rgb_sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, 
                                   depth_sub], 1, 10, allow_headerless=True)
    depth_rgb_sync.registerCallback(callback)

    img_pub = rospy.Publisher("/tennis_circle_img", Image, queue_size=10)
    pose_pub = rospy.Publisher('/tennis_pose', PoseArray, queue_size=10)    

    global get_next_point, cv_rgb, yellowLower, yellowUpper
    while(not rospy.is_shutdown()):
        if get_next_point:
            binary_image_mask = filter_color(cv_rgb, yellowLower, yellowUpper)
            contours = getContours(binary_image_mask)
            draw_ball_contour(binary_image_mask, cv_rgb ,contours, img_pub)
            transform_tennis_coordinates(contours, pose_pub)
            
        #if(cv2.waitKey(1) == ord('q')):
        #    cv2.destroyAllWindows()
        #    print("Shutting down")
        #    break

if __name__ == '__main__':
    rospy.init_node('tennis_navigation', anonymous=True)
    main()

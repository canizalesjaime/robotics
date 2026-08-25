import cv2
import numpy as np
import time

import rclpy
from rclpy.node import Node
import message_filters
import tf2_ros
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray

from intrinsic_parameters import get_intrinsic_params
from navigation_client import NavigationClient
from transformation_client import TransformClient 
from visualize import show_pose_arr

import robot_control.paths
from ros2_ws.src.robot_control.robot_control.tennis_node import TennisNavi


class TennisRos(Node):
    def __init__(self):
        super().__init__('tennis_navigation')
        self.depth_sub = message_filters.Subscriber(self, Image, "/head_camera/depth/image")
        self.rgb_sub = message_filters.Subscriber(self, Image, "/head_camera/rgb/image_raw")
        self.depth_rgb_sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, 
                                        depth_sub], 1, 10, allow_headerless=True)
        self.depth_rgb_sync.registerCallback(self.callback)
        self.img_pub = self.create_publisher(Image, "/tennis_circle_img", 10)
        self.pose_pub = self.create_publisher(PoseArray, '/tennis_pose', 10)

        self.bridge = CvBridge()
        self.cv_rgb = None # Holds messages of type Image
        self.cv_depth = None # Holds messages of type Image
        self.update_imgs= True
        self.transform_client = TransformClient()
        self.navigation_client = NavigationClient()
        self.tennis_navi = TennisNavi()


    def callback(self,ros_rgb, ros_depth):
        if self.update_imgs:
            try:
                self.cv_rgb = self.bridge.imgmsg_to_cv2(ros_rgb, "bgr8")
                self.cv_depth = self.bridge.imgmsg_to_cv2(ros_depth)
            except CvBridgeError as e:
                print(e)         


    def publish_contour(self):
        self.tennis_navi.draw_ball_contour()             
        ros_tennis_img = self.bridge.cv2_to_imgmsg(self.cv_rgb, "bgr8")
        self.img_pub.publish(ros_tennis_img)


    def publish_coordinates(self):
        self.tennis_navi.transform_tennis_coordinates()
        map_frame_pt=transform_client.transform_pt("head_camera_depth_frame", 'map', [p_X,p_Y,1])
        print("map_frame:",map_frame_pt[0],map_frame_pt[1])
        show_pose_arr(0,'map',map_frame_pt, 0, self.pose_pub)
        navigation_client.move_to_goal(map_frame_pt[0],map_frame_pt[1],Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        

def main():
    rclpy.init()
    node = TennisRos()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.process_tennis()

    except KeyboardInterrupt:
        pass
    finally:        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
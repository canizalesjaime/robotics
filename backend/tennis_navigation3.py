#!/usr/bin/env python3

###############################################################################
#
# Title          : tennis_navigation.py
# Description    : Detects a tennis ball based on its yellowness using OpenCV
#                  and computes its coordinates in the camera depth frame.
#                  Transforms the tennis ball position into the map frame and
#                  sends a navigation goal.
#
# Notes          : ROS 2 conversion of original ROS 1 node.
#
###############################################################################

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

import message_filters

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Quaternion

from cv_bridge import CvBridge

from intrinsic_parameters import get_intrinsic_params
from navigation_client import NavigationClient
from transformation_client import TransformClient
from visualize import show_pose_arr


class TennisNavigation(Node):

    def __init__(self):
        super().__init__('tennis_navigation')

        self.bridge = CvBridge()

        self.cv_rgb = None
        self.cv_depth = None
        self.get_next_point = False

        self.yellowLower = (30, 150, 100)
        self.yellowUpper = (50, 255, 255)

        #######################################################################
        # These helper classes must also be ROS 2 compatible.
        #######################################################################

        self.transform_client = TransformClient()
        self.navigation_client = NavigationClient()

        #######################################################################
        # Publishers
        #######################################################################

        self.img_pub = self.create_publisher(
            Image,
            '/tennis_circle_img',
            10
        )

        self.pose_pub = self.create_publisher(
            PoseArray,
            '/tennis_pose',
            10
        )

        #######################################################################
        # Subscribers
        #######################################################################

        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            '/head_camera/depth/image'
        )

        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            '/head_camera/rgb/image_raw'
        )

        self.depth_rgb_sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )

        self.depth_rgb_sync.registerCallback(self.callback)

        self.get_logger().info('Tennis navigation node started')


    ###########################################################################
    # RGB + depth callback
    ###########################################################################

    def callback(self, ros_rgb, ros_depth):

        try:
            self.cv_rgb = self.bridge.imgmsg_to_cv2(
                ros_rgb,
                'bgr8'
            )

            self.cv_depth = self.bridge.imgmsg_to_cv2(
                ros_depth,
                desired_encoding='passthrough'
            )

        except Exception as e:
            self.get_logger().error(
                f'CV Bridge error: {e}'
            )
            return

        #######################################################################
        # Instead of setting a flag and using a while loop,
        # process the frame directly in the callback.
        #######################################################################

        binary_image_mask = self.filter_color(
            self.cv_rgb,
            self.yellowLower,
            self.yellowUpper
        )

        contours = self.getContours(
            binary_image_mask
        )

        self.draw_ball_contour(
            binary_image_mask,
            self.cv_rgb,
            contours,
            self.img_pub
        )

        self.transform_tennis_coordinates(
            contours,
            self.pose_pub
        )


    ###########################################################################
    # Color filtering
    ###########################################################################

    def filter_color(
        self,
        rgb_image,
        lower_bound_color,
        upper_bound_color
    ):

        hsv_image = cv2.cvtColor(
            rgb_image,
            cv2.COLOR_BGR2HSV
        )

        mask = cv2.inRange(
            hsv_image,
            lower_bound_color,
            upper_bound_color
        )

        return mask


    ###########################################################################
    # Contours
    ###########################################################################

    def getContours(self, binary_image):

        contours, hierarchy = cv2.findContours(
            binary_image,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        return contours


    ###########################################################################
    # Draw detected tennis ball
    ###########################################################################

    def draw_ball_contour(
        self,
        binary_image,
        rgb_image,
        contours,
        img_pub
    ):

        black_image = np.zeros(
            [
                binary_image.shape[0],
                binary_image.shape[1],
                3
            ],
            dtype='uint8'
        )

        for c in contours:

            area = cv2.contourArea(c)

            perimeter = cv2.arcLength(
                c,
                True
            )

            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if area > 100:

                cv2.drawContours(
                    rgb_image,
                    [c],
                    -1,
                    (150, 250, 150),
                    1
                )

                cv2.drawContours(
                    black_image,
                    [c],
                    -1,
                    (150, 250, 150),
                    1
                )

                cx, cy = self.get_contour_center(c)

                cv2.circle(
                    rgb_image,
                    (cx, cy),
                    int(radius),
                    (0, 0, 255),
                    1
                )

                cv2.circle(
                    black_image,
                    (cx, cy),
                    int(radius),
                    (0, 0, 255),
                    1
                )

                cv2.circle(
                    black_image,
                    (cx, cy),
                    5,
                    (150, 150, 255),
                    -1
                )

        ros_tennis_img = self.bridge.cv2_to_imgmsg(
            rgb_image,
            'bgr8'
        )

        img_pub.publish(
            ros_tennis_img
        )


    ###########################################################################
    # Find center of contour
    ###########################################################################

    def get_contour_center(self, contour):

        M = cv2.moments(contour)

        cx = -1
        cy = -1

        if M['m00'] != 0:

            cx = int(
                M['m10'] /
                M['m00']
            )

            cy = int(
                M['m01'] /
                M['m00']
            )

        return cx, cy


    ###########################################################################
    # Convert tennis-ball pixel location into camera coordinates
    #
    # This preserves your original math as closely as possible.
    ###########################################################################

    def transform_tennis_coordinates(
        self,
        contours,
        pose_pub
    ):

        for c in contours:

            p_X = 0.0
            p_Y = 0.0
            p_Z = 0.0

            divider = 0
            distance = 0

            area = cv2.contourArea(c)

            perimeter = cv2.arcLength(
                c,
                True
            )

            ((x, y), radius) = cv2.minEnclosingCircle(c)

            int_x = int(x)
            int_y = int(y)

            intrinsic_arr = get_intrinsic_params()

            ###################################################################
            # Examine a neighborhood around the center of the detected ball.
            ###################################################################

            for i in range(
                int_x - 10,
                int_x + 10
            ):

                for j in range(
                    int_y - 10,
                    int_y + 10
                ):

                    ################################################################
                    # Added lower-bound checking so negative indexes don't wrap
                    # around the NumPy array.
                    ################################################################

                    if (
                        0 <= i < self.cv_depth.shape[1]
                        and
                        0 <= j < self.cv_depth.shape[0]
                    ):

                        depth_value = self.cv_depth[j][i]

                        if np.isfinite(depth_value):

                            distance = depth_value

                            divider += 1

                            p_X += distance

                            p_Y += (
                                -(x - intrinsic_arr[0])
                                * distance
                                / intrinsic_arr[2]
                            )

                            # Original code had this disabled:
                            #
                            # p_Z += (
                            #     -(y - intrinsic_arr[1])
                            #     * distance
                            #     / intrinsic_arr[3]
                            # )

            ###################################################################
            # Only process valid contours.
            ###################################################################

            if divider > 0 and area > 100:

                p_X = p_X / divider
                p_Y = p_Y / divider

                self.get_logger().info(
                    f'camera_frame: {p_X}, {p_Y}'
                )

                ################################################################
                # Your custom TransformClient must be converted to ROS 2.
                ################################################################

                map_frame_pt = self.transform_client.transform_pt(
                    'head_camera_depth_frame',
                    'map',
                    [p_X, p_Y, 1]
                )

                self.get_logger().info(
                    f'map_frame: '
                    f'{map_frame_pt[0]}, '
                    f'{map_frame_pt[1]}'
                )

                ################################################################
                # Visualize detected ball pose.
                ################################################################

                show_pose_arr(
                    0,
                    'map',
                    map_frame_pt,
                    0,
                    pose_pub
                )

                ################################################################
                # Send navigation goal.
                #
                # NavigationClient must also be ROS 2/Nav2 compatible.
                ################################################################

                self.navigation_client.move_to_goal(
                    map_frame_pt[0],
                    map_frame_pt[1],
                    Quaternion(
                        x=0.0,
                        y=0.0,
                        z=0.0,
                        w=1.0
                    )
                )

                break


###############################################################################
# Main
###############################################################################

def main(args=None):

    rclpy.init(args=args)

    node = TennisNavigation()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

###############################################################################
#
# Title          : tennis_navigation.py
# Description    : Detects a yellow tennis ball using OpenCV, computes its
#                  3D position from the depth image, transforms that position
#                  into the map frame, publishes the detected ball pose, and
#                  optionally sends the position to Nav2.
#
# ROS Version    : ROS 2
#
###############################################################################

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import message_filters

from sensor_msgs.msg import Image
from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
    PoseArray,
    Pose
)

from cv_bridge import CvBridge

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from nav2_simple_commander.robot_navigator import BasicNavigator


class TennisNavigation(Node):

    def __init__(self):
        super().__init__('tennis_navigation')

        #######################################################################
        # OpenCV / image parameters
        #######################################################################

        self.bridge = CvBridge()

        self.yellow_lower = np.array(
            [30, 150, 100],
            dtype=np.uint8
        )

        self.yellow_upper = np.array(
            [50, 255, 255],
            dtype=np.uint8
        )

        self.minimum_contour_area = 100.0

        #######################################################################
        # Camera intrinsic parameters
        #
        # Replace these with the actual intrinsics from your camera.
        #
        # A better future approach is subscribing to sensor_msgs/CameraInfo.
        #######################################################################

        self.fx = 525.0
        self.fy = 525.0
        self.cx = 319.5
        self.cy = 239.5

        #######################################################################
        # Frames
        #######################################################################

        self.camera_frame = 'head_camera_depth_frame'
        self.map_frame = 'map'

        #######################################################################
        # TF2
        #######################################################################

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        #######################################################################
        # Publishers
        #######################################################################

        self.image_pub = self.create_publisher(
            Image,
            '/tennis_circle_img',
            10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/tennis_pose',
            10
        )

        #######################################################################
        # RGB + depth synchronized subscribers
        #######################################################################

        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            '/head_camera/rgb/image_raw'
        )

        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            '/head_camera/depth/image'
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [
                self.rgb_sub,
                self.depth_sub
            ],
            queue_size=10,
            slop=0.1
        )

        self.sync.registerCallback(
            self.camera_callback
        )

        #######################################################################
        # Nav2
        #######################################################################

        self.navigator = BasicNavigator()

        # We do NOT want to send a new goal for every camera frame.
        self.navigation_goal_sent = False

        self.get_logger().info(
            'Tennis navigation node started.'
        )


    ###########################################################################
    # Camera callback
    ###########################################################################

    def camera_callback(self, rgb_msg, depth_msg):

        try:

            rgb_image = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding='bgr8'
            )

            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding='passthrough'
            )

        except Exception as e:

            self.get_logger().error(
                f'CV Bridge conversion failed: {e}'
            )

            return

        #######################################################################
        # Detect yellow objects
        #######################################################################

        binary_mask = self.filter_color(
            rgb_image
        )

        contours = self.get_contours(
            binary_mask
        )

        #######################################################################
        # Find best tennis ball candidate
        #######################################################################

        ball_contour = self.select_ball_contour(
            contours
        )

        if ball_contour is None:

            self.publish_debug_image(
                rgb_image,
                rgb_msg
            )

            return

        #######################################################################
        # Find center of ball in image
        #######################################################################

        center = self.get_contour_center(
            ball_contour
        )

        if center is None:
            return

        u, v = center

        #######################################################################
        # Draw detected tennis ball
        #######################################################################

        self.draw_ball_contour(
            rgb_image,
            ball_contour,
            center
        )

        self.publish_debug_image(
            rgb_image,
            rgb_msg
        )

        #######################################################################
        # Get depth around center
        #######################################################################

        depth = self.get_average_depth(
            depth_image,
            u,
            v
        )

        if depth is None:

            self.get_logger().debug(
                'No valid depth measurement for tennis ball.'
            )

            return

        #######################################################################
        # Pixel -> 3D point in camera frame
        #######################################################################

        camera_point = self.pixel_to_camera_point(
            u,
            v,
            depth,
            depth_msg.header.stamp
        )

        self.get_logger().info(
            f'Ball in camera frame: '
            f'x={camera_point.point.x:.3f}, '
            f'y={camera_point.point.y:.3f}, '
            f'z={camera_point.point.z:.3f}'
        )

        #######################################################################
        # Camera frame -> map frame
        #######################################################################

        map_point = self.transform_to_map(
            camera_point
        )

        if map_point is None:
            return

        self.get_logger().info(
            f'Ball in map frame: '
            f'x={map_point.point.x:.3f}, '
            f'y={map_point.point.y:.3f}, '
            f'z={map_point.point.z:.3f}'
        )

        #######################################################################
        # Publish ball pose
        #######################################################################

        ball_pose = self.point_to_pose(
            map_point
        )

        self.pose_pub.publish(
            ball_pose
        )

        #######################################################################
        # Send Nav2 goal
        #######################################################################

        if not self.navigation_goal_sent:

            self.send_navigation_goal(
                ball_pose
            )


    ###########################################################################
    # Color filtering
    ###########################################################################

    def filter_color(self, rgb_image):

        hsv_image = cv2.cvtColor(
            rgb_image,
            cv2.COLOR_BGR2HSV
        )

        binary_mask = cv2.inRange(
            hsv_image,
            self.yellow_lower,
            self.yellow_upper
        )

        return binary_mask


    ###########################################################################
    # Contours
    ###########################################################################

    def get_contours(self, binary_image):

        contours, _ = cv2.findContours(
            binary_image,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        return contours


    def select_ball_contour(self, contours):

        valid_contours = []

        for contour in contours:

            area = cv2.contourArea(
                contour
            )

            if area >= self.minimum_contour_area:
                valid_contours.append(
                    contour
                )

        if not valid_contours:
            return None

        #######################################################################
        # For now:
        # assume the largest yellow contour is the tennis ball.
        #######################################################################

        return max(
            valid_contours,
            key=cv2.contourArea
        )


    def get_contour_center(self, contour):

        moments = cv2.moments(
            contour
        )

        if moments['m00'] == 0:
            return None

        cx = int(
            moments['m10'] /
            moments['m00']
        )

        cy = int(
            moments['m01'] /
            moments['m00']
        )

        return cx, cy


    ###########################################################################
    # Drawing
    ###########################################################################

    def draw_ball_contour(
        self,
        rgb_image,
        contour,
        center
    ):

        cx, cy = center

        (_, _), radius = cv2.minEnclosingCircle(
            contour
        )

        cv2.drawContours(
            rgb_image,
            [contour],
            -1,
            (150, 250, 150),
            2
        )

        cv2.circle(
            rgb_image,
            (cx, cy),
            int(radius),
            (0, 0, 255),
            2
        )

        cv2.circle(
            rgb_image,
            (cx, cy),
            5,
            (255, 0, 0),
            -1
        )


    def publish_debug_image(
        self,
        rgb_image,
        original_msg
    ):

        try:

            output_msg = self.bridge.cv2_to_imgmsg(
                rgb_image,
                encoding='bgr8'
            )

            output_msg.header = original_msg.header

            self.image_pub.publish(
                output_msg
            )

        except Exception as e:

            self.get_logger().error(
                f'Failed to publish debug image: {e}'
            )


    ###########################################################################
    # Depth
    ###########################################################################

    def get_average_depth(
        self,
        depth_image,
        u,
        v,
        radius=10
    ):

        height, width = depth_image.shape[:2]

        x_min = max(
            0,
            u - radius
        )

        x_max = min(
            width,
            u + radius + 1
        )

        y_min = max(
            0,
            v - radius
        )

        y_max = min(
            height,
            v + radius + 1
        )

        depth_region = depth_image[
            y_min:y_max,
            x_min:x_max
        ]

        #######################################################################
        # Remove NaNs / invalid depth
        #######################################################################

        valid_depth = depth_region[
            np.isfinite(depth_region)
        ]

        valid_depth = valid_depth[
            valid_depth > 0
        ]

        if valid_depth.size == 0:
            return None

        #######################################################################
        # Depending on the camera, depth may arrive as:
        #
        # 32FC1 -> meters
        # 16UC1 -> millimeters
        #
        #######################################################################

        depth = float(
            np.median(valid_depth)
        )

        if depth_image.dtype == np.uint16:

            depth /= 1000.0

        return depth


    ###########################################################################
    # Camera projection
    ###########################################################################

    def pixel_to_camera_point(
        self,
        u,
        v,
        depth,
        stamp
    ):

        #######################################################################
        # Standard pinhole camera model:
        #
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        # Z = depth
        #
        #######################################################################

        x = (
            (u - self.cx)
            * depth
            / self.fx
        )

        y = (
            (v - self.cy)
            * depth
            / self.fy
        )

        z = depth

        point = PointStamped()

        point.header.stamp = stamp
        point.header.frame_id = self.camera_frame

        point.point.x = x
        point.point.y = y
        point.point.z = z

        return point


    ###########################################################################
    # TF transformation
    ###########################################################################

    def transform_to_map(
        self,
        camera_point
    ):

        try:

            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                camera_point.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )

            map_point = do_transform_point(
                camera_point,
                transform
            )

            return map_point

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ) as e:

            self.get_logger().warning(
                f'TF transform failed: {e}'
            )

            return None


    ###########################################################################
    # Point -> Pose
    ###########################################################################

    def point_to_pose(
        self,
        point
    ):

        pose = PoseStamped()

        pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        pose.header.frame_id = (
            self.map_frame
        )

        pose.pose.position.x = (
            point.point.x
        )

        pose.pose.position.y = (
            point.point.y
        )

        #######################################################################
        # Nav2 generally treats the robot as moving in a planar environment.
        #######################################################################

        pose.pose.position.z = 0.0

        #######################################################################
        # Identity quaternion
        #######################################################################

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        return pose


    ###########################################################################
    # Navigation
    ###########################################################################

    def send_navigation_goal(
        self,
        goal_pose
    ):

        self.get_logger().info(
            'Sending tennis ball position to Nav2.'
        )

        try:

            self.navigator.goToPose(
                goal_pose
            )

            self.navigation_goal_sent = True

        except Exception as e:

            self.get_logger().error(
                f'Failed to send Nav2 goal: {e}'
            )


###############################################################################
# Main
###############################################################################

def main(args=None):

    rclpy.init(
        args=args
    )

    tennis_node = TennisNavigation()

    try:

        rclpy.spin(
            tennis_node
        )

    except KeyboardInterrupt:

        pass

    finally:

        tennis_node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
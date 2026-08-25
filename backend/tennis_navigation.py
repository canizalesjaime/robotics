# 
# Title          : tennis_navigation.py
# Author         : 
# Created on     :    
# Description    : Detects a tennis ball based on it yellowness using opencv      
#                  and computes its coordinates in the camera depth frame. 
#                  Creates a new frame for the tennis ball with  
#                  head_camera_depth_frame being its parent frame.      
# Usage          : 
# Modifications  : 
# Notes          : 
#                 
# 
###############################################################################
import cv2
import numpy as np
import time


class TennisNavi():
    def __init__(self):
        self.yellowLower =(30, 150, 100)
        self.yellowUpper = (50, 255, 255)
        

    def filter_color(self):
        hsv_image = cv2.cvtColor(self.cv_rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, self.yellowLower, self.yellowUpper)
        return mask


    def getContours(self, hsv_image):      
        contours, hierarchy = cv2.findContours(hsv_image, 
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
        return contours


    def draw_ball_contour(self,binary_image, contours, img_pub):
        black_image = np.zeros([binary_image.shape[0], 
                                binary_image.shape[1],3],'uint8')
        
        for c in contours:
            area = cv2.contourArea(c)
            perimeter= cv2.arcLength(c, True)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if (area>100):
                cv2.drawContours(self.cv_rgb, [c], -1, (150,250,150), 1)
                cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
                cx, cy = self.get_contour_center(c)
                cv2.circle(self.cv_rgb, (cx,cy),(int)(radius),(0,0,255),1)
                cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
                cv2.circle(black_image, (cx,cy),5,(150,150,255),-1)
                print ("Area: {}, Perimeter: {}".format(area, perimeter))
        print ("number of contours: {}".format(len(contours)))
        cv2.imshow("RGB Image Contours",self.cv_rgb)
        cv2.waitKey(0)
        cv2.imshow("Black Image Contours",black_image)
        cv2.waitKey(1)


    def get_contour_center(self,contour):
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
    def transform_tennis_coordinates(self,contours):        
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
            #intrinsic_arr = get_intrinsic_params()
            for i in range( int_x - 10, int_x + 10 ):
                for j in range( int_y - 10, int_y + 10 ):
                    if  (0 <= i < 640 and 0 <= j < 480) and not np.isnan(cv_depth[j][i]):
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
                
                map_frame_pt=transform_client.transform_pt("head_camera_depth_frame", 'map', [p_X,p_Y,1])
                print("map_frame:",map_frame_pt[0],map_frame_pt[1])
                show_pose_arr(0,'map',map_frame_pt, 0, self.pose_pub)
                navigation_client.move_to_goal(map_frame_pt[0],map_frame_pt[1],Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                break


    def process_tennis(self):
        if self.cv_rgb is not None:
            self.update_imgs = False
            hsv_mask = self.filter_color()
            contours = self.getContours(hsv_mask)
            self.draw_ball_contour(hsv_mask, contours)
            #self.transform_tennis_coordinates(contours)
            self.update_imgs = True


def main():
    node = TennisNavi()
    try:
        while True:
            node.process_tennis()
            if(cv2.waitKey(1) == ord('q')):
                cv2.destroyAllWindows()
                print("Shutting down")
                break

    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main()
    
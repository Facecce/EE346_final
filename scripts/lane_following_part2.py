#!/usr/bin/env python
import math
from collections import deque
from datetime import datetime
from time import sleep
from webbrowser import get
import cv2
import cv_bridge
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


ARUCO_DICT = {"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL}

class Follower:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('camera/image',
                                          Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                                           Twist, queue_size=10)

        self.twist = Twist()
        self.velocity = 0.3
        self.turns=0
        self.mode=1
        self.previous_mode=1

    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, d = image.shape
   
        warped = cv2.warpPerspective(image, M, (w, h))
        hsv2 = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        
        gray = cv2.cvtColor(warped,cv2.COLOR_BGR2GRAY)

        lower_black=np.array([0,0,20])
        upper_black=np.array([180,255,90])
        mask = cv2.inRange(hsv2, lower_black, upper_black)
        mask4 = cv2.inRange(hsv2, lower_black, upper_black)

        moments=[]
        center_list = []
        # line detect
        ima, contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP,
                                                  cv2.CHAIN_APPROX_NONE) 
        for i, cnt in enumerate(contours):
            center, radius = cv2.minEnclosingCircle(contours[i])
            perimeter = cv2.arcLength(cnt, True)          
            if(perimeter>100):
                m=cv2.moments(contours[i])
                moments.append(m)
                center_list.append((int(center[0]), int(center[1]), cnt))
        for i in center_list:
            cv2.circle(warped, (i[0], i[1]), 2, (0, 0, 255), 3)
            cv2.drawContours(warped,i[2],-1,(0,255,0),2)

        if(len(moments)>=1 and len(moments)<=3):
        # if M3['m00'] and M4['m00'] > 0:
            if len(moments)==1:
                Mx=moments[0]
                cx= int(Mx['m10']/Mx['m00'])
                cy=int(Mx['m01']/Mx['m00'])
                fpt_x=cx/2
                self.mode=1
                rospy.loginfo('mode is'+str(self.mode))
            elif len(moments)==2 :
                M3=moments[0]
                M4=moments[1]
                cx1 = int(M3['m10']/M3['m00'])
                cy1 = int(M3['m01']/M3['m00'])
                cx2 = int(M4['m10']/M4['m00'])
                cy2 = int(M4['m01']/M4['m00'])
                fpt_x = (cx1 + cx2)/2
                fpt_y = (cy1 + cy2)/2 + 2*h/3
                cv2.circle(warped, (cx1, cy1), 10, (0, 255, 255), -1)
                cv2.circle(warped, (cx2, cy2), 10, (255, 255, 255), -1)
                cv2.circle(warped, (fpt_x, fpt_y), 10, (128, 128, 128), -1)
                self.mode=1
                rospy.loginfo('mode is'+str(self.mode))
            else :
                Coner1=moments[0]
                Coner2=moments[1]
                line=moments[2]
                cx1 = int(Coner1['m10']/Coner1['m00'])
                cy1 = int(Coner1['m01']/Coner1['m00'])
                cx2 = int(Coner2['m10']/Coner2['m00'])
                cy2 = int(Coner2['m01']/Coner2['m00'])
                # cx3 = int(line['m10']/line['m00'])
                # cy3 = int(line['m01']/line['m00'])
                cv2.circle(warped, (cx1, cy1), 10, (0, 255, 255), -1)
                cv2.circle(warped, (cx2, cy2), 10, (255, 255, 255), -1)
                # cv2.circle(warped, (cx3, cy3), 10, (128, 128, 128), -1)
                
                if (cx1>w/2 and cx2>w/2 ):
                    if(self.turns>=3):
                        fpt_x=(cx1+cx2)/2
                    else :fpt_x=w/2
                    self.mode=2
                    
                    rospy.loginfo("end")
                elif ((cx1>w/2 and cx2<w/2) or (cx1<w/2 and cx2>w/2)):
                    fpt_x=cx1
                    rospy.loginfo("start")
                else:fpt_x=w/2-30
                
            # rospy.loginfo(len(moments))
            err = w/2 - fpt_x
            # rospy.loginfo(err)
            self.twist.linear.x = self.velocity
            self.twist.angular.z = err*3/80
            if (self.previous_mode==2and self.mode==1):
                self.turns+=1
            self.previous_mode=self.mode
            rospy.loginfo('turn is'+str(self.turns))
            rospy.loginfo('mode is'+str(self.mode))
            rospy.loginfo('previous_mode is'+str(self.previous_mode))
            
            
            # rospy.loginfo(self.twist.angularx.z)
            self.cmd_vel_pub.publish(self.twist)

        cv2.imshow("window", image)
        cv2.imshow("BEV", warped)
        cv2.imshow("mask",mask)
        cv2.waitKey(1)


rospy.init_node('lane_follower')
# M=numpy.array([[-7.24044334e-01,-1.33589686e+00 ,2.75194752e+02],
#  [ 5.88035368e-16,-3.09726306e+00 ,5.01191812e+02],
#  [ 1.88257696e-18,-8.36914725e-03 ,1.00000000e+00]])

# M=numpy.array([[-3.32435282e-01,-1.08187888e+00 ,1.76936863e+02],
#  [ 2.59220803e-15,-3.35902542e+00 ,5.53364648e+02],
#  [ 9.18312741e-18,-8.55101324e-03 ,1.00000000e+00]])

#   H_0.65= [[ -7.24044334e-01  -1.33589686e+00   2.75194752e+02]
#                    [  5.88035368e-16  -3.20696675e+00   5.27410993e+02]
#                   [  2.80521771e-18  -8.36914725e-03   1.00000000e+00]]


# H_0. 5=[[ -7.24044334e-01  -1.33589686e+00   2.75194752e+02]
#                     [  5.88035368e-16  -3.09726306e+00   5.01191812e+02]
#                     [  1.88257696e-18  -8.36914725e-03   1.00000000e+00]]


# H_0.732=[[ -7.24044334e-01  -1.33589686e+00   2.75194752e+02]
#                     [  9.80058946e-16  -3.27278896e+00   5.43142502e+02]
#                     [  2.34437961e-18  -8.36914725e-03   1.00000000e+00]]

#H matrix
# M = numpy.array([[ -7.24044334e-01 ,-1.33589686e+00 , 2.75194752e+02],
#                     [  5.88035368e-16 ,-3.09726306e+00  ,5.01191812e+02],
#                     [  1.88257696e-18 ,-8.36914725e-03  ,1.00000000e+00]])

M=np.array( [[ -7.24044334e-01 ,-1.33589686e+00 , 2.75194752e+02],
                   [  5.88035368e-16 ,-3.20696675e+00  ,5.27410993e+02],
                  [  2.80521771e-18 ,-8.36914725e-03  ,1.00000000e+00]])

follower = Follower()
rospy.spin()



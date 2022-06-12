#!/usr/bin/env python
import math
from collections import deque
from datetime import datetime
from time import sleep

import cv2
from numpy import vectorize
import cv_bridge
import numpy
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
        self.velocity = 0

    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = numpy.array([20, 100, 100])
        upper_yellow = numpy.array([34, 255, 250])

        lower_white = numpy.array([0, 0, 100])
        upper_white = numpy.array([180, 43, 220])

        mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask2 = cv2.inRange(hsv, lower_white, upper_white)
        def gethsv(event,x,y,flags,param):
            if event==cv2.EVENT_LBUTTONDOWN:
                rospy.loginfo(hsv2[y,x])

        h, w, d = image.shape
        search_top = 2*h/3
        serach_top2 = h/4
        mask1[0:search_top, 0:w] = 0
        mask2[0:search_top, 0:w] = 0

        M1 = cv2.moments(mask1)
        M2 = cv2.moments(mask2)

        warped = cv2.warpPerspective(image, M, (w, h))

        hsv2 = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        # mask3 = cv2.inRange(hsv2, lower_yellow, upper_yellow)
        # mask4 = cv2.inRange(hsv2, lower_white, upper_white)
        # mask3[0:0, 0:w] = 0
        # mask4[0:0, 0:w] = 0
        # M3 = cv2.moments(mask3)
        # M4 = cv2.moments(mask4)



        cv2.imshow("window", image)
        cv2.imshow("BEV", warped)
        cv2.setMouseCallback("BEV",gethsv)
        cv2.waitKey(1)


rospy.init_node('lane_follower')
# M=numpy.array([[-7.24044334e-01,-1.33589686e+00 ,2.75194752e+02],
#  [ 5.88035368e-16,-3.09726306e+00 ,5.01191812e+02],
#  [ 1.88257696e-18,-8.36914725e-03 ,1.00000000e+00]])

# M=numpy.array([[-3.32435282e-01,-1.08187888e+00 ,1.76936863e+02],
#  [ 2.59220803e-15,-3.35902542e+00 ,5.53364648e+02],
#  [ 9.18312741e-18,-8.55101324e-03 ,1.00000000e+00]])



#H matrix
M = numpy.array([[-7.24044334e-01, -1.33589686e+00, 2.75194752e+02],
                 [9.80058946e-16, -3.27278896e+00, 5.43142502e+02],
                 [2.34437961e-18, -8.36914725e-03, 1.00000000e+00]])

follower = Follower()
rospy.spin()



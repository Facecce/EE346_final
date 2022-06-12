#!/usr/bin/env python
import math
from collections import deque
from datetime import datetime
from time import sleep
import cv2
import cv_bridge
import numpy 
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from turtlebot3_msgs.msg import turns


desired_aruco_dictionary = "DICT_6X6_50"
# The different ArUco dictionaries built into the OpenCV library.
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
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
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

# dist=numpy.array(([[-0.58650416 , 0.59103816, -0.00443272 , 0.00357844 ,-0.27203275]]))
# newcameramtx=numpy.array([[189.076828   ,  0.    ,     361.20126638]
#  ,[  0 ,2.01627296e+04 ,4.52759577e+02]
#  ,[0, 0, 1]])
# mtx=numpy.array([[398.12724231  , 0.      ,   304.35638757],
#  [  0.       ,  345.38259888, 282.49861858],
#  [  0.,           0.,           1.        ]])

dist=numpy.array(([[-0.2909375306628219, 0.05890305811963341, 0.002023707366213156, 0.002460957243230047, 0]]))
newcameramtx=numpy.array([[189.076828   ,  0.    ,     361.20126638]
 ,[  0 ,2.01627296e+04 ,4.52759577e+02]
 ,[0, 0, 1]])
mtx=numpy.array([[255.69528   ,  0.    ,     162.32018]
 ,[  0 ,256.3211 ,133.64144]
 ,[0, 0, 1]])


class Follower:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('camera/image',
                                          Image, self.image_callback)
        # self.turns_sub=rospy.Subscriber('turns_num',turns,self.turns_cb)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                                           Twist, queue_size=1)

        self.twist = Twist()
        self.distance=0
        self.velocity=0.3
        self.turns=0
        self.mode=1
        self.previous_mode=1
        self.last_err=0
        self.init_time=datetime.now()
        self.time_test=datetime.now()
        self.last_time_test=datetime.now()

    def turns_cb(self,msg):
        self.turns=msg.num
    def aruco_detect(self, image):
        # global distance
        this_aruco_dictionary = cv2.aruco.Dictionary_get(
            ARUCO_DICT[desired_aruco_dictionary])
        this_aruco_parameters = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
            image, this_aruco_dictionary, parameters=this_aruco_parameters)

        if len(corners) > 0:
            # rospy.loginfo("the distance is %f",distance)
            ids = ids.flatten()
            rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(
                corners, 0.05, mtx, dist)
            (rvec-tvec).any()
            for i in range(rvec.shape[0]):
                cv2.aruco.drawAxis(
                    image, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.05)
                cv2.aruco.drawDetectedMarkers(image, corners)
            cv2.putText(image, "Id: " + str(ids), (0, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            deg = rvec[0][0][2]/math.pi*180
            R = numpy.zeros((3, 3), dtype=numpy.float64)
            cv2.Rodrigues(rvec, R)
            sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
            singular = sy < 1e-6
            if not singular:
                x = math.atan2(R[2, 1], R[2, 2])
                y = math.atan2(-R[2, 0], sy)
                z = math.atan2(R[1, 0], R[0, 0])
            else:
                x = math.atan2(-R[1, 2], R[1, 1])
                y = math.atan2(-R[2, 0], sy)
                z = 0
            rx = x * 180.0 / 3.141592653589793
            ry = y * 180.0 / 3.141592653589793
            rz = z * 180.0 / 3.141592653589793

            cv2.putText(image, 'deg_z:'+str(ry)+str('deg'), (0, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            self.distance = (tvec[0][0][2] + 0.085)*100

            cv2.putText(image, 'distance:' + str(round(self.distance, 4)) + str('m'),
                        (0, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return len(corners) > 0

    def shutdown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

    def image_callback(self, msg):
        global count
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, d = image.shape
   
        warped = cv2.warpPerspective(image, M, (w, h))
        hsv2 = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        
        gray = cv2.cvtColor(warped,cv2.COLOR_BGR2GRAY)

        lower_black=numpy.array([0,0,20])
        upper_black=numpy.array([180,255,80])
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

        detect=self.aruco_detect(image)
        print(str(h)+"      "+str(w))
        if(len(moments)>=1 and len(moments)<=3):
        # if M3['m00'] and M4['m00'] > 0:
            if len(moments)==1:
                print("only one")
                Mx=moments[0]
                cx= int(Mx['m10']/Mx['m00'])
                cy=int(Mx['m01']/Mx['m00'])
                print(cy)
                print(cx-w/2)
                if  cy<h/2 and (datetime.now()-self.init_time).total_seconds()<10:
                    fpt_x=w/2+50
                else:
                    fpt_x=cx/2
                self.mode=0
                

            elif len(moments)==2 :
                M3=moments[0]
                M4=moments[1]
                cx=[]
                cy=[]
                cx1 = int(M3['m10']/M3['m00'])
                cx.append(cx1)
                cy1 = int(M3['m01']/M3['m00'])
                cy.append(cy1)
                cx2 = int(M4['m10']/M4['m00'])
                cx.append(cx2)
                cy2 = int(M4['m01']/M4['m00'])
                cy.append(cy2)
                cx.sort()
                cy.sort()
                fpt_x = (cx1 + cx2)/2
                fpt_y = (cy1 + cy2)/2 + 2*h/3
                if(cx[1]==cx1):
                    cy[1]=cy1
                    cy[0]=cy2
                else:
                    cy[1]=cy2
                    cy[0]=cy1
                
                print(str(cx[1])+"   "+str(cy[1]))
                print(str(cx[0])+"   "+str(cy[0]))
                if (cx[1]>240 and cy[1]>190) or (cy[1]<100 and cx[1]>250):
                    if(cx[0]<100 and abs(cy[0]-h/2)<20):
                        self.mode=2
                        if self.turns>=3:
                            if cy[1]>205:
                                fpt_x=cx[1]+50
                            # err=w/2-fpt_x
                            # self.twist.linear.x = 0.2
                            # self.twist.angular.z = err*3/80
                            # self.cmd_vel_pub.publish(self.twist)
                                rospy.sleep(0.06)
                            else:fpt_x=w/2
                        if(self.previous_mode==0):
                            self.last_time_test=self.time_test
                            self.time_test=datetime.now()
                            if(self.time_test-self.last_time_test).total_seconds()>3:
                                self.turns+=1

                else:self.mode=0
                
                cv2.circle(warped, (cx1, cy1), 10, (0, 255, 255), -1)
                cv2.circle(warped, (cx2, cy2), 10, (255, 255, 255), -1)
                cv2.circle(warped, (fpt_x, fpt_y), 10, (128, 128, 128), -1)


            else :
                Coner1=moments[0]
                Coner2=moments[1]
                line=moments[2]
                cx=[]
                cy=[]
                cx1 = int(Coner1['m10']/Coner1['m00'])
                cx.append(cx1)
                cy1 = int(Coner1['m01']/Coner1['m00'])
                cy.append(cy1)
                cx2 = int(Coner2['m10']/Coner2['m00'])
                cx.append(cx2)
                cy2 = int(Coner2['m01']/Coner2['m00'])
                cy.append(cy2)
                cx3 = int(line['m10']/line['m00'])
                cx.append(cx3)
                cy3 = int(line['m01']/line['m00'])
                cy.append(cy3)
                cx.sort()
                cy.sort()
                rospy.loginfo(cx)
                rospy.loginfo(cy)
                cv2.circle(warped, (cx1, cy1), 10, (0, 255, 255), -1)
                cv2.circle(warped, (cx2, cy2), 10, (255, 255, 255), -1)
                # cv2.circle(warped, (cx3, cy3), 10, (128, 128, 128), -1)

                # if ((cx[2]>w/2+80 and cx[1]>w/2+80) and (cy[2]>h/2+20 and abs(cy[1]-h/2)<30) ):
                #     self.mode=2
                #     if(self.turns>=2):
                #         rospy.loginfo(str(cx[2])+"and"+str(cx[1]))
                #         fpt_x=w/2+75
                #     else :
                #         fpt_x=w/2
                #         if (self.previous_mode==0 and self.mode==2):
                #             self.turns+=1
                #     # if((cy1+cy2+cy3)/3==h/2):
                #     rospy.loginfo("end")
                # else:
                fpt_x=w/2
                self.mode=1

            err = w/2 - fpt_x
            self.previous_mode=self.mode
            rospy.loginfo("mode is "+str(self.mode))
            rospy.loginfo("turns is "+str(self.turns))
            if detect and 15<self.distance <40 :
                    if self.distance<16:
                        self.shutdown()
                        start_time = datetime.now()
                        rospy.sleep(5)
                        rospy.loginfo("stop")
                        end_time = datetime.now()
                        durn = (end_time-start_time).total_seconds()
                        rospy.loginfo("Total time =%.4f", durn)
                    else:
                             self.velocity=0.1
                             self.twist.linear.x = self.velocity
                             self.twist.angular.z = 0  
                             self.cmd_vel_pub.publish(self.twist)
            else:

                self.twist.linear.x = 0.2
                # self.twist.angular.z = err*Kp+Kd*(err-self.last_err)
                self.twist.angular.z=err*3/80
                self.last_err=err
                
                # self.cmd_vel_pub.publish(self.twist)
                
            cv2.imshow("window", image)
            cv2.imshow("BEV", warped)
            cv2.imshow("mask",mask)
            cv2.waitKey(1)


rospy.init_node('lane_follower')

M = numpy.array([[-7.24044334e-01, -1.33589686e+00, 2.75194752e+02],
                 [9.80058946e-16, -3.27278896e+00, 5.43142502e+02],
                 [2.34437961e-18, -8.36914725e-03, 1.00000000e+00]])

count = 0
distance = 0
Kp=2.5/80
Kd=0.12
follower = Follower()
rospy.spin()



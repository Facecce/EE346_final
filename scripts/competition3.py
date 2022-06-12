#!/usr/bin/env python
from cv2 import sqrt
import rospy, cv2, cv_bridge
import numpy,math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult
import time
import pyttsx
from datetime import datetime
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
dist=numpy.array(([[-0.2909375306628219, 0.05890305811963341, 0.002023707366213156, 0.002460957243230047, 0]]))
newcameramtx=numpy.array([[189.076828   ,  0.    ,     361.20126638]
 ,[  0 ,2.01627296e+04 ,4.52759577e+02]
 ,[0, 0, 1]])
mtx=numpy.array([[255.69528   ,  0.    ,     162.32018]
 ,[  0 ,256.3211 ,133.64144]
 ,[0, 0, 1]])

class Competiton:

        def __init__(self):
                self.point=1
                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('camera/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)

                self.pose_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

                self.result_sub=rospy.Subscriber('move_base/result',MoveBaseActionResult,self.result_cb)

                self.twist = Twist()

                self.rate = rospy.Rate(50) 
                self.distance=0
                self.id=0
                self.detect=True
                self.initial=0
                self.mypose=PoseStamped()
                self.engine=pyttsx.init()
                self.engine.setProperty('rate', 150)
                
        def shutdown(self):
            rospy.loginfo("Shutting down. cmd_vel will be 0")
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)

        def result_cb(self,msg):
            # print(msg)
            
            if(msg.status.status==3):
                rospy.loginfo("reach")  
                # start_time=datetime.now()
                # if self.point==4:
                #     while(self.detect==False):
                #         self.twist.linear.x =0
                #         self.twist.angular.z=1
                #         self.cmd_vel_pub.publish(self.twist)
                #         time=datetime.now()
                #         if (time-start_time).total_seconds()>3:
                #             break
                # self.shutdown()
                # rospy.sleep(1)
                self.point+=1
                self.pose_publish()


        def image_callback(self, msg):
                    global  M,count
                    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                    this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
                    this_aruco_parameters = cv2.aruco.DetectorParameters_create()
                    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, this_aruco_dictionary, parameters=this_aruco_parameters)

                    if len(corners) > 0:
                                self.detect=True
                                ids = ids.flatten()
                                self.id=ids
                                rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
                                (rvec-tvec).any()
                                for i in range(rvec.shape[0]):
                                        cv2.aruco.drawAxis(image, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.05)
                                        cv2.aruco.drawDetectedMarkers(image, corners)
                                cv2.putText(image, "Id: " + str(ids), (0,40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)        
                                deg=rvec[0][0][2]/math.pi*180
                                R=numpy.zeros((3,3),dtype=numpy.float64)
                                cv2.Rodrigues(rvec,R)
                                sy=math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
                                singular=sy< 1e-6
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

                                cv2.putText(image,'deg_z:'+str(ry)+str('deg'),(0, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                                distance = (tvec[0][0][2] + 0.085)*100  
                                cv2.putText(image, 'distance:' + str(round(distance, 4)) + str('m'), (0, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                                rospy.loginfo("yes, the id is :" +str(ids))	
                                self.engine.say(str(ids))
                                self.engine.runAndWait()
                    else: 
                        rospy.loginfo("no")
                        self.detect=False
                    cv2.imshow("window", image)
                    cv2.waitKey(1)
        def main(self):
            while not rospy.is_shutdown():
                if self.initial==0:
                    self.mypose.header.frame_id='map'
                    self.mypose.pose.position.x=3.89705848694
                    self.mypose.pose.position.y= 1.79976141453
                    self.mypose.pose.position.z=0.0
                    self.mypose.pose.orientation.x=0
                    self.mypose.pose.orientation.y=0
                    self.mypose.pose.orientation.z=0.630159199169
                    self.mypose.pose.orientation.w=0.776465957852
                    self.pose_pub.publish(self.mypose)
                    self.initial=1
                    self.rate.sleep()
            rospy.loginfo("first")
        def pose_publish(self):
                        if self.point ==1:
                            self.mypose=PoseStamped()
                            self.mypose.header.frame_id='map'
                            self.mypose.pose.position.x=3.89705848694
                            self.mypose.pose.position.y= 1.79976141453
                            self.mypose.pose.position.z=0.0
                            self.mypose.pose.orientation.x=0
                            self.mypose.pose.orientation.y=0
                            self.mypose.pose.orientation.z=0.630159199169
                            self.mypose.pose.orientation.w=0.776465957852
                            status=0
                        elif self.point==2:
                            # self.mypose=PoseStamped()
                            # turtle_vel_pub.publish(self.mypose) 
                            self.mypose=PoseStamped()
                            self.mypose.header.frame_id='map'
                            self.mypose.pose.position.x=0.356288671494
                            self.mypose.pose.position.y= -0.126959264278
                            self.mypose.pose.position.z=0.0
                            self.mypose.pose.orientation.x=0
                            self.mypose.pose.orientation.y=0
                            self.mypose.pose.orientation.z=0
                            self.mypose.pose.orientation.w=1
                            # self.point=0
                            status=0

                        elif self.point==3:
                            # self.mypose=PoseStamped()
                            # turtle_vel_pub.publish(self.mypose) 
                            self.mypose=PoseStamped()
                            self.mypose.header.frame_id='map'
                            self.mypose.pose.position.x=2.26973342896
                            self.mypose.pose.position.y=-3.71048784256
                            self.mypose.pose.position.z=0.0
                            self.mypose.pose.orientation.x=0
                            self.mypose.pose.orientation.y=0
                            self.mypose.pose.orientation.z=0
                            self.mypose.pose.orientation.w=1
                            status=0

                        # elif self.point==4:
                        #     # self.mypose=PoseStamped()
                        #     # turtle_vel_pub.publish(self.mypose) 
                        #     self.mypose=PoseStamped()
                        #     self.mypose.header.frame_id='map'
                        #     self.mypose.pose.position.x=2.03492069244
                        #     self.mypose.pose.position.y=-2.88090610504
                        #     self.mypose.pose.position.z=0.0
                        #     self.mypose.pose.orientation.x=0
                        #     self.mypose.pose.orientation.y=0
                        #     self.mypose.pose.orientation.z=0
                        #     self.mypose.pose.orientation.w=1
                        #     status=0                        

                        elif self.point==4:
                            # self.mypose=PoseStamped()
                            # turtle_vel_pub.publish(self.mypose) 
                            self.mypose=PoseStamped()
                            self.mypose.header.frame_id='map'
                            self.mypose.pose.position.x=5.87116527557
                            self.mypose.pose.position.y=-1.72441291809
                            self.mypose.pose.position.z=0.0
                            self.mypose.pose.orientation.x=0
                            self.mypose.pose.orientation.y=0
                            self.mypose.pose.orientation.z=0.999004570943
                            self.mypose.pose.orientation.w=1
                            status=0
                            self.point=0
                            # rospy.loginfo("navigation is over!")
    
                        self.pose_pub.publish(self.mypose)


rospy.init_node('Aruco_detect')
count=0
competition=Competiton()


rospy.spin()

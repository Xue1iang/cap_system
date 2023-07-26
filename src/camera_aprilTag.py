#!/usr/bin/env python3
import tf
import rospy
import math
import numpy as np
from tf import TransformBroadcaster
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped, Pose
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagCornersArray
import datetime
import time
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R


class CameraApriltag:

    def __init__(self):
        # ----- Initialization of all translation and rotation -----
        
        # Read the static transformation t_B_C
        self.r_B_C = np.array(rospy.get_param("r_B_C"))
        # convert string to float
        # self.r_B_C = (float(r_B_C[1]), float(r_B_C[3]), float(r_B_C[5]), float(r_B_C[7])) 
        self.t_B_C = np.array(rospy.get_param("t_B_C"))
        # convert string to float
        # self.t_B_C = (float(t_B_C[1]), float(t_B_C[3]), -float(t_B_C[6]))
        
        self.intrinsic = np.array(rospy.get_param("intrinsic_matrix"))
        self.t_apriltag = (0,0,0)
        self.r_apriltag = (0,0,0,0)
        self.centre = np.array((0,0,0))
        self.t_c_p = ([0], [0], [0])

  
        # ------ publishers --------
        self.pub = rospy.Publisher('/projected_in_camera', geometry_msgs.msg.Pose, queue_size=1)
        self.pub_one_input = geometry_msgs.msg.Pose()


         # ------ rate and shutdown ------
        self.rate = rospy.Rate(60.0)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)


    # ------ publisher for clean shutdown ------
    def publisher_def(self, pub_one_input):
        while not self.ctrl_c:
            self.pub.publish(pub_one_input)
            rospy.loginfo("System is shutting down. Stopping to send information from camera......")
            break

    # ----- shutdown stuff ---------
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        
        self.stop()
        self.ctrl_c = True
        
    def stop(self):
        # rospy.loginfo("shutdown hook")
        
        # put safe shutdown commands here
        self.pub_one_input.position.x = 0
        self.pub_one_input.position.y = 0
        self.pub_one_input.position.z = 0

        self.pub_one_input.orientation.x = 0
        self.pub_one_input.orientation.y = 0
        self.pub_one_input.orientation.z = 0
        self.pub_one_input.orientation.w = 1

        self.publisher_def(self.pub_one_input)  # publish zero translation and rotation information when you hit ctrl-c


    def callback_apriltag(self, apriltag):
        self.t_apriltag = (apriltag.detections[0].pose.pose.pose.position.x, apriltag.detections[0].pose.pose.pose.position.y, apriltag.detections[0].pose.pose.pose.position.z)
        self.r_apriltag = (apriltag.detections[0].pose.pose.pose.orientation.x, apriltag.detections[0].pose.pose.pose.orientation.y, apriltag.detections[0].pose.pose.pose.orientation.z, apriltag.detections[0].pose.pose.pose.orientation.w)

    def callback_corners(self,pixel_centre):
        self.centre = np.array([pixel_centre.corners[0].centre.x, pixel_centre.corners[0].centre.y, 0])


    def main(self, even=None):

        rospy.Subscriber('/tag_corners', AprilTagCornersArray, self.callback_corners)
        rospy.Subscriber('/tag_detection',AprilTagCornersArray, self.callback_apriltag)


        while not self.ctrl_c:


            self.t_c_p = np.matmul(np.array(inv(self.intrinsic)), np.array([[self.centre[0]], [self.centre[1]], [1]]))
            print(self.t_c_p)
    
            self.h_cp = geometry_msgs.msg.Pose()
            self.h_cp.position.x = self.t_c_p[0]
            self.h_cp.position.y = self.t_c_p[0]
            self.h_cp.position.z = 1
    
            self.h_cp.orientation.w = 1
            self.h_cp.orientation.x = 0
            self.h_cp.orientation.y = 0
            self.h_cp.orientation.z = 0
            self.pub.publish(self.h_cp)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('cam_apriltag')
    ca = CameraApriltag()
    try:
        ca.main()
    except rospy.ROSInterruptException:
        pass
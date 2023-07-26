#!/usr/bin/env python3
import tf
import rospy
import math
import tf
import numpy as np
from tf import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg
import datetime
import time
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R
import sys
import xc_defs
from std_msgs.msg import Float64

from queue import Queue

class XueliangPositioning:

    def __init__(self):

        #----- Initialisation of transformation -----
        # Read the static transformation t_B_C
        r_B_C = rospy.get_param('r_B_C')
        # convert string to float
        self.r_B_C = (float(r_B_C[1]), float(r_B_C[3]), float(r_B_C[5]), float(r_B_C[7])) 
        t_B_C = rospy.get_param('t_B_C')
        # convert string to float
        self.t_B_C = (float(t_B_C[1]), float(t_B_C[3]), -float(t_B_C[6]))
        # Initialize Vicon data (will be IMU and SLAM later)
        self.t_W_B = (0,0,0)
        self.r_W_B = (0,0,0,1)
        # Initialize camera's data
        self.t_C_P = (0,0,0)
        self.r_C_P = (0,0,0,1)
        # Initialize pressure sensor's data
        self.pressure_sens = 0
        # Initialize Vicon input data
        self.t_W_B = (0,0,0)
        self.r_W_B = (0,0,0,1)
        self.t_W_M = (0,0,0)
        
        rospy.Subscriber('/vicon/mallard_nov2021_2/mallard_nov2021_2', TransformStamped, self.callback_w2b)
        rospy.Subscriber('/pressure_sensor', Float64, self.callback_ps)
        rospy.Subscriber('/projected_in_camera', geometry_msgs.msg.Pose, self.callback_spoofcam)
        
    def callback_w2b(self, w2b):

        self.t_W_B = (w2b.transform.translation.x, w2b.transform.translation.y, w2b.transform.translation.z)
        self.r_W_B = (w2b.transform.rotation.x, w2b.transform.rotation.y, w2b.transform.rotation.z, w2b.transform.rotation.w)
        
        # Caculation of t_W_M using all provided sensors infomation 
        self.t_W_M = xc_defs.x_positioning(self.pressure_sens, self.t_W_B, self.r_W_B, self.t_B_C, self.r_B_C, self.t_C_P, self.r_C_P)

        # tf in rviz (marker related to world frame)
        tf_W_M = TransformBroadcaster()
        tf_W_M.sendTransform((self.t_W_M[0], self.t_W_M[1], self.t_W_M[2]),  (0,0,0,1), rospy.Time.now(), 'MMMMMMMMMMMMMM', 'world')        
        
        h_wm = geometry_msgs.msg.Pose()
        h_wm.position.x = self.t_W_M[0]
        h_wm.position.y = self.t_W_M[1]
        h_wm.position.z = self.t_W_M[2]

        h_wm.orientation.w = 1
        h_wm.orientation.x = 0
        h_wm.orientation.y = 0
        h_wm.orientation.z = 0

        pub = rospy.Publisher('/predicted_marker', geometry_msgs.msg.Pose, queue_size=10)
        pub.publish(h_wm)  

    def callback_ps(self, ps):
        self.pressure_sens = ps.data  


    def callback_spoofcam(self, aruco):
        self.t_C_P = (aruco.position.x, aruco.position.y, aruco.position.z)
        self.r_C_P = (aruco.orientation.x, aruco.orientation.y, aruco.orientation.z, aruco.orientation.w)


if __name__ == '__main__':
    rospy.init_node('xueliang_positioning', anonymous=True)
    try:
        XueliangPositioning()
    except rospy.ROSInterruptException:
        pass
   
    rospy.spin()









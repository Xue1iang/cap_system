#! /usr/bin/env python3

from tf import TransformBroadcaster
import rospy
from rospy import Time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Pose
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagCornersArray
import numpy as np
from numpy.linalg import inv
import math
from std_msgs.msg import Float32
from scipy.spatial.transform import Rotation as R
import sys
import xc_defs
from std_msgs.msg import Float64
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped

class XueliangPositioning:

    def __init__(self):

        #----- Initialisation of transformation -----
        # Read the static transformation t_B_C
        self.r_B_C = np.array(rospy.get_param('r_B_C'))
        # convert string to float
        # self.r_B_C = (float(r_B_C[1]), float(r_B_C[3]), float(r_B_C[5]), float(r_B_C[7])) 
        self.t_B_C = np.array(rospy.get_param('t_B_C'))
        # convert string to float
        # self.t_B_C = (float(t_B_C[1]), float(t_B_C[3]), -float(t_B_C[6]))
        # Initialize Vicon data (will be IMU and SLAM later)
        self.t_w_b_slam = (0,0,0)
        self.r_w_b_slam = (0,0,0,1)
        # Initialize camera's data
        self.t_C_P = (0,0,0)
        self.r_C_P = (0,0,0,1)
        # Initialize pressure sensor's data
        self.pressure_sens = 0
        # Initialize Vicon input data
        # self.t_W_B = (0,0,0)
        # self.r_W_B = (0,0,0,1)
        self.t_W_M = (0,0,0)
        
        # rospy.Subscriber('/nav/filtered_imu/data', Imu, self.callback_alignedimu)
        rospy.Subscriber('/imu_alignment', Imu, self.callback_alignedimu)
        rospy.Subscriber('/slam_out_pose', PoseStamped, self.callback_slam)
        rospy.Subscriber('/pressure_sensor', Float64, self.callback_ps)
        rospy.Subscriber('/projected_in_camera', geometry_msgs.msg.Pose, self.callback_camAprilTag)
        

    def callback_slam(self, slam):
        self.t_w_b_slam = (slam.pose.position.x, slam.pose.position.y, slam.pose.position.z)
        self.r_w_b_slam = (slam.pose.orientation.x, slam.pose.orientation.y, slam.pose.orientation.z, slam.pose.orientation.w)

    
    def callback_camAprilTag(self, camAprilTag):
        self.t_C_P = (camAprilTag.position.x, camAprilTag.position.y, camAprilTag.position.z)
        self.r_C_P = (camAprilTag.orientation.x, camAprilTag.orientation.y, camAprilTag.orientation.z, camAprilTag.orientation.w)

    def callback_ps(self, ps):
        self.pressure_sens = ps.data 
    
    
    def callback_alignedimu(self, imu_slamyaw):
        

        self.r_W_B = (imu_slamyaw.orientation.x, imu_slamyaw.orientation.y, imu_slamyaw.orientation.z, imu_slamyaw.orientation.w)
        
        
        self.t_W_M = xc_defs.x_positioning(self.pressure_sens, self.t_w_b_slam, self.r_W_B, self.t_B_C, self.r_B_C, self.t_C_P, self.r_C_P)

        # tf in rviz (predicted marker related to world frame)
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


if __name__ == '__main__':
    rospy.init_node('xueliang_positioning', anonymous=True)
    try:
        XueliangPositioning()
    except rospy.ROSInterruptException:
        pass
   
    rospy.spin()    
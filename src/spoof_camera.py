#!/usr/bin/env python3
import tf
import rospy
import math
import tf
import numpy as np
from tf import TransformBroadcaster
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
import datetime
import time
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R


 
class SpoofCam:

    def __init__(self):
        # ----- Initialization of all translation and rotation -----
        
        # Read the static transformation t_B_C
        r_B_C = rospy.get_param('r_B_C')
        # convert string to float
        self.r_B_C = (float(r_B_C[1]), float(r_B_C[3]), float(r_B_C[5]), float(r_B_C[7])) 
        t_B_C = rospy.get_param('t_B_C')
        # convert string to float
        self.t_B_C = (float(t_B_C[1]), float(t_B_C[3]), -float(t_B_C[6]))
        self.t_W_B = (0,0,0)
        self.r_W_B = (0,0,0,1)
        self.tx_C_P = 0
        self.ty_C_P = 0
        self.tz_C_P = 0
        self.H_W_M = ([[1,0,0,0],
                       [0,1,0,0],
                       [0,0,1,-3],
                       [0,0,0,1]]) # This is a assumption for spoof camera data only

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
            rospy.loginfo("System is shutting down. Stopping sending information from camera......")
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



    def callback_w2b(self, w2b):

        self.t_W_B = (w2b.transform.translation.x, w2b.transform.translation.y, w2b.transform.translation.z)
        self.r_W_B = (w2b.transform.rotation.x, w2b.transform.rotation.y, w2b.transform.rotation.z, w2b.transform.rotation.w)


    def qt2H(self, r, t):
        # quaternion and translation to homogenous transformation matrix
        r = R.from_quat(r)
        rotm = r.as_matrix()
        H = ([[rotm[0,0], rotm[0,1], rotm[0,2], t[0]],
              [rotm[1,0], rotm[1,1], rotm[1,2], t[1]],
              [rotm[2,0], rotm[2,1], rotm[2,2], t[2]],
              [0,         0,         0,         1   ]])
        
        return H


    def main(self, even=None):


        rospy.Subscriber('/vicon/mallard_nov2021_2/mallard_nov2021_2', TransformStamped, self.callback_w2b)

        while not self.ctrl_c:

            # H_W_C = H_W_B * H_B_C
            H_W_B = self.qt2H(self.r_W_B, self.t_W_B)           
            H_B_C = self.qt2H(self.r_B_C, self.t_B_C)
            
            H_W_C = np.matmul(H_W_B, H_B_C)

            # H_C_M = H_W_C_^-1 * H_W_M
            H_C_M_sim = np.matmul(inv(np.array(H_W_C)), np.array(self.H_W_M))

            r_wc = R.from_matrix([[H_W_C[0,0], H_W_C[0,1],H_W_C[0,2]],
                                  [H_W_C[1,0], H_W_C[1,1],H_W_C[1,2]],
                                  [H_W_C[2,0], H_W_C[2,1],H_W_C[2,2]]])
            r_W_C_forward = r_wc.as_quat()

            r_cm = R.from_matrix([[H_C_M_sim[0,0], H_C_M_sim[0,1],H_C_M_sim[0,2]],
                                  [H_C_M_sim[1,0], H_C_M_sim[1,1],H_C_M_sim[1,2]],
                                  [H_C_M_sim[2,0], H_C_M_sim[2,1],H_C_M_sim[2,2]]])
            r_C_M_sim = r_cm.as_quat()
            
            H_C_P = H_C_M_sim
            self.tx_C_P = H_C_P[0,3] / H_C_P[2,3]
            self.ty_C_P = H_C_P[1,3] / H_C_P[2,3]
            self.tz_C_P = H_C_P[2,3] / H_C_P[2,3]

            # ----- tfs in rviz -----
            # world frame to camera frame
            tf_W_C = TransformBroadcaster()
            tf_W_C.sendTransform((H_W_C[0,3], H_W_C[1,3], H_W_C[2,3]), r_W_C_forward, rospy.Time.now(), 'camera_forward', 'world')

            # camera frame to marker frame
            tf_C_M = TransformBroadcaster()
            tf_C_M.sendTransform((H_C_M_sim[0,3], H_C_M_sim[1,3], H_C_M_sim[2,3]), r_C_M_sim, rospy.Time.now(), 'marker_sim', 'camera_forward')

            # Camera frame to P (projected marker on scaled plane of camera)
            tf_C_P = TransformBroadcaster()
            tf_C_P.sendTransform((self.tx_C_P, self.ty_C_P, self.tz_C_P), (0,0,0,1), rospy.Time.now(), 'P', 'camera_forward')

            # Publish the spoof data (t_C_P)
            self.h_cp = geometry_msgs.msg.Pose()
            self.h_cp.position.x = self.tx_C_P
            self.h_cp.position.y = self.ty_C_P
            self.h_cp.position.z = self.tz_C_P

            self.h_cp.orientation.w = 1
            self.h_cp.orientation.x = 0
            self.h_cp.orientation.y = 0
            self.h_cp.orientation.z = 0
            self.pub.publish(self.h_cp)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('aruco')
    sc = SpoofCam()
    try:
        sc.main()
    except rospy.ROSInterruptException:
        pass
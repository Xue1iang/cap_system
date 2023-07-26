#!/usr/bin/env python3
import tf
import rospy
import math
import tf
import numpy as np
from tf import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Imu
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R
import xc_defs


class ImuAlignment:


    def __init__(self):

        self.counter = 1

        # Initialize IMU alignment data
        self.r_imu_b_static = np.array(rospy.get_param("r_I_B"))

        self.r_imu_reading = (0,0,0,1)


        self.r_w_b_slam = (0,0,0,1)
        self.t_w_b_slam = (0,0,0) 


        # ------ publishers and subscribers --------
        self.aligned_imu_publisher = rospy.Publisher('/imu_alignment', Imu, queue_size=1)
        self.pub_one_input = Imu()


         # ------ rate and shutdown ------
        self.rate = rospy.Rate(80.0)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)


    # ------ publisher for clean shutdown ------
    def publisher_def(self, pub_one_input):
        while not self.ctrl_c:
            self.aligned_imu_publisher.publish(pub_one_input)
            rospy.loginfo("System is shutting down. Stopping sending information from IMU alignment block......")
            break

    # ----- shutdown stuff ---------
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        
        self.stop()
        self.ctrl_c = True
        
    def stop(self):
        # rospy.loginfo("shutdown hook")
        
        # put safe shutdown commands here
        self.pub_one_input.orientation.x =0
        self.pub_one_input.orientation.y =0
        self.pub_one_input.orientation.z =0
        self.pub_one_input.orientation.w =1
        self.publisher_def(self.pub_one_input)  # publish zero translation and rotation information when you hit ctrl-c


    def callback_imu(self, imu):

        self.r_imu_reading = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)

    def callback_slam(self, slam):
        
        self.counter +=1

        self.t_w_b_slam = (slam.pose.position.x, slam.pose.position.y, slam.pose.position.z)
        self.r_w_b_slam = (slam.pose.orientation.x, slam.pose.orientation.y, slam.pose.orientation.z, slam.pose.orientation.w)


        r_w_sb = R.from_quat(self.r_w_b_slam)

        r_imu_b_ = R.from_quat(self.r_imu_b_static)
        r_imu_b_conj = r_imu_b_.inv()    

        r_arb_imu_ = R.from_quat(self.r_imu_reading)
        r_arb_imu_conj = r_arb_imu_.inv()

        r_w_arb_temp = r_w_sb * r_imu_b_conj * r_arb_imu_conj # self.r_w_arb_temp is <class 'scipy.spatial.transform.rotation.Rotation'>
        
        # self.r_w_arb_temp = self.r_w_arb_temp.as_quat() # (x, y, z, w)

            
        r_w_arb_avg_temp = r_w_arb_temp


        q_r_w_arb_avg_temp = r_w_arb_avg_temp.as_quat()
        # print(type(self.r_w_arb_avg_temp))
        q_r_w_arb_temp = r_w_arb_temp.as_quat()
        # print(q_r_w_arb_temp.shape)
        Q = np.array([[q_r_w_arb_avg_temp[3], q_r_w_arb_avg_temp[0], q_r_w_arb_avg_temp[1], q_r_w_arb_avg_temp[2]],[q_r_w_arb_temp[3], q_r_w_arb_temp[0], q_r_w_arb_temp[1], q_r_w_arb_temp[2]]])
        w =np.array(((self.counter -1)/self.counter, 1/self.counter))
            

        r_w_arb_avg_temp = xc_defs.weightedAverageQuaternions(Q, w) # self.r_w_arb_avg_temp is 'numpy.ndarray' and in order of (w,x,y,z)
        qq_r_w_arb_avg_temp = np.array([r_w_arb_avg_temp[1], r_w_arb_avg_temp[2], r_w_arb_avg_temp[3], r_w_arb_avg_temp[0]])
        # print(qq_r_w_arb_avg_temp)
        r_w_arb_avg_temp = R.from_quat(qq_r_w_arb_avg_temp)
        # print(type(self.r_w_arb_avg_temp))

    #     # # self.r_w_arb_avg_temp_ = R.from_quat(self.r_w_arb_avg_temp)
        r_w_b_ = r_w_arb_avg_temp * r_arb_imu_ * r_imu_b_
        r_w_b_array  = np.array(r_w_b_.as_quat())
            
        # print(self.r_w_b_array.shape)

        r_w_b = Imu()
        r_w_b.orientation.w = r_w_b_array[3]
        r_w_b.orientation.x = r_w_b_array[0]
        r_w_b.orientation.y = r_w_b_array[1]
        r_w_b.orientation.z = r_w_b_array[2]

        self.aligned_imu_publisher.publish(r_w_b)


    def main(self, even=None):

        rospy.Subscriber('/nav/filtered_imu/data', Imu, self.callback_imu)
        rospy.Subscriber('/slam_out_pose', PoseStamped, self.callback_slam)

        while not self.ctrl_c:

            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('imu_alinment', anonymous=True)
    im = ImuAlignment()
    try:
        im.main()
    except rospy.ROSInterruptException:
        pass
 
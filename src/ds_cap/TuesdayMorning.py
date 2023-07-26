#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import Imu
import xc_defs
from geometry_msgs.msg import PoseStamped, Pose
from apriltag_ros.msg import AprilTagDetectionRawArray
import tf
from std_msgs.msg import Float64
from tf import TransformBroadcaster

class TiltingEKF:

    def __init__(self):
 
        # initialize angular velocity
        angular_vel_x = np.deg2rad(10.0)
        angular_vel_y = np.deg2rad(10.0)
        angular_vel_z = np.deg2rad(0.0)
        self.gyro_init = np.array([[angular_vel_x], [angular_vel_y], [angular_vel_z]])
        self.decision_margin = 0
        # Initialize the depth sensor
        self.depth = 0
        # Initialize slam euler angles
        self.slameul = np.array([0,0,0])
        
        # Initialize
        self.t_w2sb = (0,0,0)
        self.r_w2sb = (0,0,0,1)

        # Initialize the t_c2p
        self.t_c2p = np.array([0,0,0])
        self.r_c2p = np.array([0,0,0,1])

        self.t_w2m = np.array([0,0,0])

        # 初始化存储最新传感器读数的列表
        self.latest_values = [0, 0, 0, 0]
        self.lost_count_slam = 0
        self.lost_count_depth = 0
        self.lost_count_camera = 0
        self.lost_count_imu = 0
        # Read the static transformation t_B_C
        r_B_C = rospy.get_param('r_B_C')
        # convert string to float
        self.r_b2c = np.array([float(r_B_C[0]), float(r_B_C[1]), float(r_B_C[2]), float(r_B_C[3])]) 
        
        t_B_C = rospy.get_param('t_B_C')
        # convert string to float
        self.t_b2c = (float(t_B_C[0]), float(t_B_C[1]), -float(t_B_C[2]))

        period_ms = rospy.get_param('period_ms')
        self.dts = period_ms / 30

        self.r_i2b = rospy.get_param('r_I_B')
        # Initialize the ds_cap (x,y,z)

        # Initialise the counter
        # self.lost_count = 0
        # Initialise the centre of tag (pxiel coordinates)
        self.t_centre = (0,0)

        #
        self.t_C_P = np.array([[0],[0],[0]])
        # Intrinsic matrix (3x3)
        self.intrinsic_mat = np.array(rospy.get_param('intrinsic_matrix'))


        # observation matrix H
        self.H = np.diag([1.0, 1.0])

        # system noise variance
        self.Q = np.diag([0.01962, 0.034516])

        # observation noise variance
        self.R = np.diag([0.0000179,0.0000179])

        # initialize status
        # self.x_true = np.array([[1.43], [0.03]])
        self.x_true = np.array([[0.0], [0.0]])

        # initialize prediction
        self.x_bar = self.x_true

        # initialize eastimation
        self.x_hat = self.x_true

        # initialize covariance
        self.P = self.Q

        # initialize jacbian matrix of H
        self.jacobianH = np.diag([1.0, 1.0])
        # ------ publishers --------
        # self.pub = rospy.Publisher('/projected_in_camera', geometry_msgs.msg.PoseStamped, queue_size=1)
        self.pub_dscap = rospy.Publisher('/dscap_sys666', PoseStamped, queue_size=10)
        self.pub_one_input = Pose()

        # ------ subscriber -------
        self.depth_subscriber = rospy.Subscriber('/cap_depth', Float64, self.callback_depthSensor)
        self.slam_subscriber = rospy.Subscriber('/cap_slam', PoseStamped, self.callback_slam)
        # self.camera_subscriber = rospy.Subscriber('/projected_in_camera', PoseStamped, self.callback_camera)
        rospy.Subscriber('/cap_t_c2p', PoseStamped, self.callback_cam)
        
        self.imu_subscriber = rospy.Subscriber('/cap_imu', Imu, self.callback_imu)
        # self.imu_subscriber = rospy.Subscriber('/nav/filtered_imu/data', Imu, self.callback_imu)

        # ------ rate and shutdown ------
        self.rate = rospy.Rate(60.0)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    # ------ publisher for clean shutdown ------
    def publisher_def(self, pub_one_input):
        while not self.ctrl_c:
            self.pub_dscap.publish(pub_one_input)
            rospy.loginfo("CAP-DS system is shutting down......")
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

    def callback_imu(self, imu):
        # IMU triggers depth seneor, SLAM and camera(AprilTag)' input.
        self.r_w2b = np.array([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])        
        
        # Rviz for SLAM
        tf_W_B = TransformBroadcaster()
        tf_W_B.sendTransform((self.t_w2sb[0], self.t_w2sb[1], self.t_w2sb[2]), (self.r_w2b[0], self.r_w2b[1], self.r_w2b[2], self.r_w2b[3]), rospy.Time.now(), 'MallARD_003', 'World')

        # Run the main cap function
        self.t_w2m = xc_defs.ds_cap(self.depth, self.t_w2sb, self.r_w2b, self.t_b2c, self.r_b2c, self.t_c2p, self.r_c2p)           

        dsCAP = PoseStamped()
        dsCAP.header.stamp = rospy.Time.now()
        dsCAP.header.frame_id = 'BlueROV2_base_link'

        dsCAP.pose.position.x = self.t_w2m[0]
        dsCAP.pose.position.y = self.t_w2m[1]
        dsCAP.pose.position.z = self.t_w2m[2]
        self.pub_dscap.publish(dsCAP)


        # Rviz for ds_cap
        tf_W_M = TransformBroadcaster()
        tf_W_M.sendTransform((self.t_w2m[0], self.t_w2m[1], self.t_w2m[2]),  (0,0,0,1), rospy.Time.now(), 'BlueROV2', 'World')        

    def callback_slam(self,w2sb):
        # SLAM translation
        self.t_w2sb = np.array([w2sb.pose.position.x, w2sb.pose.position.y, w2sb.pose.position.z])


    def callback_depthSensor(self, depth):
        self.depth = depth.data

    def callback_cam(self, pixed_centre):
        self.t_c2p = np.array([pixed_centre.pose.position.x, pixed_centre.pose.position.y, pixed_centre.pose.position.z])


    def ekf_update(self, gyro_angular_vel, accel_data, jacobianH):
        # [step1] prediction
        self.x_bar = xc_defs.get_status(self.x_hat, gyro_angular_vel, self.dts)
        
        # jacobian matrix           
        jacobianF = xc_defs.get_jacobianF(self.x_bar, gyro_angular_vel, self.dts)
        # pre_covariance
        P_bar = jacobianF @ self.P @ jacobianF.T + self.Q

        # observation
        roll = np.arctan2(accel_data[1],accel_data[2])
        pitch = np.arctan2(accel_data[0], np.sqrt(accel_data[1]**2 + accel_data[2]**2))
        y = np.array([roll, pitch])
        # print(y)
        # [step2] update the filter
        s = self.H @ P_bar @ self.H.T + self.R
        K = (P_bar @ self.H.T) @ np.linalg.inv(s)

        # eastimation
        e = y - (jacobianH @ self.x_bar)
        x_hat = self.x_bar + (K @ e)
        # post_covariance
        I = np.eye(x_hat.shape[0])
        P = (I - K @ self.H) @ P_bar

        # EKF output
        self.x_hat = x_hat
        # Covariance
        self.P = P
        
        return self.x_hat
    

if __name__ == '__main__':

    rospy.init_node('ds_cap')
    try:
        te = TiltingEKF()
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
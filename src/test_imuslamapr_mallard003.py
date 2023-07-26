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

class Xueliang:

    def __init__(self, intrinsic_matrix):
        self.intrinsic = intrinsic_matrix
        self.corners = np.array([[0,0], [0,0], [0,0], [0,0]])
        self.z_height_measured = 1.80

    global rotation_imu, translation_slam, rotation_slam, translation_apr, rotation_apr

    rotation_imu = (0,0,0,0)
    translation_slam = (0,0,0)
    rotation_slam = (0,0,0,0)
    translation_apr = (0,0,0)
    rotation_apr = (0,0,0,0)

   
    def callback_apriltag(self, apriltag):
        global translation_apr, rotation_apr
        translation_apr = (apriltag.detections[0].pose.pose.pose.position.x, apriltag.detections[0].pose.pose.pose.position.y, apriltag.detections[0].pose.pose.pose.position.z)
        rotation_apr = (apriltag.detections[0].pose.pose.pose.orientation.x, apriltag.detections[0].pose.pose.pose.orientation.y, apriltag.detections[0].pose.pose.pose.orientation.z, apriltag.detections[0].pose.pose.pose.orientation.w)


    def callback_imu(self, imu):
        global rotation_imu
        # imu_frame = TransformBroadcaster()
        # translation1 = (0.5,0,0)
        rotation_imu = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)

   
        # imu_frame.sendTransform(translation1,rotation1,rospy.Time.now(), 'imu_3dmgq7', 'usb_cam')
   
    def callback_slam(self, slam):
        global translation_slam
        global rotation_slam
       
        translation_slam = (slam.pose.position.x, slam.pose.position.y, slam.pose.position.z)
        rotation_slam = (slam.pose.orientation.x, slam.pose.orientation.y, slam.pose.orientation.z, slam.pose.orientation.w)

    def callback_corners(self,pixel_corners):
        self.corners = np.array([[pixel_corners.corners[0].top_left.x, pixel_corners.corners[0].top_left.y], [pixel_corners.corners[0].top_right.x, pixel_corners.corners[0].top_right.y], [pixel_corners.corners[0].bottom_left.x, pixel_corners.corners[0].bottom_left.y], [pixel_corners.corners[0].bottom_right.x, pixel_corners.corners[0].bottom_right.y]])


    def quatrotate(self, q0,q1,q2,q3,x,y,z):
        q = np.array([[1-2*pow(q1,2)-2*pow(q2,2), 2*(q0*q1+q3*q2),  2*(q0*q2-q3*q1)],
                      [2*(q0*q1-q3*q2), 1-2*pow(q0,2)-2*pow(q2,2), 2*(q1*q2+q3*q0)],
                      [2*(q0*q2+q3*q1), 2*(q1*q2-q3*q0), 1-2*pow(q0,2)-2*pow(q1,2)]])

        v = np.array([x,y,z])
        v = np.transpose(v)

        r = np.matmul(q,v)
        return r

   
    def quaternion_rotation_matrix(self,Q):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
   
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)
   
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix.
                 This rotation matrix converts a point in the local reference
                 frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[3]
        q1 = Q[0]
        q2 = Q[1]
        q3 = Q[2]

        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)

        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)

        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1

        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                               [r10, r11, r12],
                               [r20, r21, r22]])

        return rot_matrix

    def quaternion_multiply(self,Q0,Q1):

        """
        Multiplies two quaternions.
   
        Input
        :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31)
        :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32)
   
        Output
        :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)
   
        """
        # Extract the values from Q0
        w0 = Q0[3]
        x0 = Q0[0]
        y0 = Q0[1]
        z0 = Q0[2]
       
        # Extract the values from Q1
        w1 = Q1[3]
        x1 = Q1[0]
        y1 = Q1[1]
        z1 = Q1[2]
       
        # Computer the product of the two quaternions, term by term
        Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
        Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
        Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
        Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
       
        # Create a 4 element array containing the final quaternion
        final_quaternion = np.array([Q0Q1_x, Q0Q1_y, Q0Q1_z, Q0Q1_w])
       
        # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
        return final_quaternion


    def quatconj(self,q):
        # q is an array
        qw = q[3]
        qx = q[0]
        qy = q[1]
        qz = q[2]

        quat_conj = np.array([-qx, -qy, -qz, qw])

        return quat_conj


    def main(self):

        rospy.init_node('my_imu')
           
        rate = rospy.Rate(30) # 30Hz
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback_apriltag )
        rospy.Subscriber('/nav/filtered_imu/data', Imu, self.callback_imu)
        rospy.Subscriber('/slam_out_pose', PoseStamped, self.callback_slam)
        rospy.Subscriber('/tag_corners', AprilTagCornersArray, self.callback_corners)
        publisher = rospy.Publisher('xueliang_', Pose, queue_size=1)
        publisher_world = rospy.Publisher('xueliang_world', Pose, queue_size=1)
        publisher_imu = rospy.Publisher('xueliang_imu', Imu, queue_size=1)
   
        while not rospy.is_shutdown():

            world_frame = TransformBroadcaster()

            '''
            q_imuinit2cam: [0, 1, 0, 0]
            q_imu2cam:  [0,0.7071,0.7071,0]
            q_imu2imuinit: [0.7071, 0, 0, 0.7071]
            q_mal0032imuinit: [0,1,0,0]
            rotation_c2mallard003: [1,0,0,0]
            rotation_mal0032c: [1,0,0,0]

            '''
            translation_w2lidar = np.array(translation_slam)

            q_imuinit2cam = np.array(rospy.get_param('q_imuinit2cam'))
            q_imu2cam = np.array(rospy.get_param('q_imu2cam'))
            q_imu2imuinit = np.array(rospy.get_param('q_imu2imuinit'))
            q_mal0032imuinit = np.array(rospy.get_param('q_mal0032imuinit'))
            rotation_c2mallard003 = np.array(rospy.get_param('rotation_c2mallard003'))
            rotation_mal0032c = np.array(rospy.get_param('rotation_mal0032c'))

            # Align the IMU frame to mallard_003 body_link frame
            a = self.quaternion_multiply(rotation_imu, self.quatconj(np.array([-0.7071,-0,-0,0.7071]))) # IMU reading in initial z-pointing-down frame

            # Align imuinit to mallard_003 body_link frame
            b = self.quaternion_multiply(a, q_mal0032imuinit)
            c = self.quaternion_multiply(np.array([-0,-1,-0,0]),b) # Align imuinit to mallard_003 body_link frame
            world_frame.sendTransform(translation_w2lidar, c, rospy.Time.now(), 'mallard_003_baselink', 'world_fixed')

            mallard0032c = TransformBroadcaster()
            translation_mal0032c = (-0.12,0,0)
            mallard0032c.sendTransform(translation_mal0032c, rotation_mal0032c, rospy.Time.now(), 'cam', 'mallard_003_baselink')

            # imu_frame = TransformBroadcaster()
            # translation_c2i = (0,0,-0.1)
            # rotation_c2i = (0,0.7071,0.7071,0)
            # imu_frame.sendTransform(translation_c2i,rotation_c2i,rospy.Time.now(), 'imu_3dmgq7', 'cam')

            # el2ca_ = TransformBroadcaster()
            # translation_e2c = (-0.08,0,0)
            # rotation_e2c = (0,0,-0.7071,0.7071)
            # el2ca_.sendTransform(translation_e2c, rotation_e2c, rospy.Time.now(), 'usb_cam', 'el_mallard_baselink')

            aprTag = TransformBroadcaster()

            camera_x_apr = translation_apr[0]
            camera_y_apr = translation_apr[1]
            camera_z_apr = translation_apr[2]

            # Calculating the transformation from apriltag frame to mallard body_link frame
            xyz_mal003_apr = self.quatrotate(rotation_c2mallard003[0], rotation_c2mallard003[1], rotation_c2mallard003[2], rotation_c2mallard003[3], camera_x_apr, camera_y_apr, camera_z_apr)

            # rotation_tag2mallard_baselink = self.quaternion_multiply(rotation_c2mallard003, rotation_apr)

            # Calculating the transformation from apriltag frame to mallard stabilized body_link frame
            a = self.quaternion_multiply(rotation_imu, self.quatconj(np.array([-0.7071,-0,-0,0.7071]))) # IMU reading in initial z-pointing-down frame
            b = self.quaternion_multiply(a, q_mal0032imuinit)
            c = self.quaternion_multiply(np.array([-0,-1,-0,0]),b) # align imuinit to mallard frame
            print(c)

            c_conj = self.quatconj(c)
            xyz_mal003_stabilized_apr = self.quatrotate(c_conj[0],c_conj[1],c_conj[2],c_conj[3], xyz_mal003_apr[0], xyz_mal003_apr[1],xyz_mal003_apr[2])

            # Calculating the translation from apriltag_frame to slam_map frame(which could be world fixed frame)
            xyz_map_apr = np.array(translation_slam) + np.array(xyz_mal003_stabilized_apr)
            aprTag.sendTransform(xyz_map_apr, np.array([0,0,0,1]), rospy.Time.now(), 'aprilTag_detection', 'world_fixed')

            rate.sleep()

if __name__ == '__main__':
    intrinsic_matrix = np.array(rospy.get_param("intrinsic_matrix"))
    andy = Xueliang(intrinsic_matrix)
    andy.main()

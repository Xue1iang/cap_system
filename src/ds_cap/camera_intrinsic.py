#! /usr/bin/env python3
import tf
import rospy
import numpy as np
from tf import TransformBroadcaster
from apriltag_ros.msg import AprilTagDetectionRawArray
import geometry_msgs.msg
from std_msgs.msg import Header

class CameraIntrinsic:

    def __init__(self):
        # Initialise the counter
        self.lost_count = 0
        # Initialise the centre of tag (pxiel coordinates)
        self.t_centre = (0,0)

        #
        self.t_C_P = np.array([[0],[0],[0]])
        # Intrinsic matrix (3x3)
        self.intrinsic_mat = np.array(rospy.get_param('intrinsic_matrix'))
        # print(intrinsic_mat.shape)


        # ------ Subscriber --------
        rospy.Subscriber('/tag_detections_raw', AprilTagDetectionRawArray, self.callback_cam)


        # ------ publishers --------
        self.pub = rospy.Publisher('/cap_ds/projected_in_camera', geometry_msgs.msg.PoseStamped, queue_size=1)


         # ------ rate and shutdown ------
        self.rate = rospy.Rate(60.0)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    # ------ publisher for clean shutdown ------
    def publisher_def(self, pub_one_input):
        while not self.ctrl_c:
            # rospy.loginfo("System is shutting down. Stopping to send information from camera......")
            break

    # ----- shutdown stuff ---------
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def callback_cam(self, pixed_centre):
        if pixed_centre.detections:
            self.t_centre = np.array([pixed_centre.detections[0].centre.x, pixed_centre.detections[0].centre.y, pixed_centre.detections[0].centre.z])
            self.t_C_P = np.linalg.inv(self.intrinsic_mat) @ ([self.t_centre[0]],[self.t_centre[1]],[1])


            self.h_cp = geometry_msgs.msg.PoseStamped()
            self.h_cp.header = Header()
            self.h_cp.header.stamp = rospy.Time.now() 
            self.h_cp.pose.position.x = self.t_C_P[0]
            self.h_cp.pose.position.y = self.t_C_P[1]
            self.h_cp.pose.position.z = 1

            self.h_cp.pose.orientation.w = 1
            self.h_cp.pose.orientation.x = 0
            self.h_cp.pose.orientation.y = 0
            self.h_cp.pose.orientation.z = 0

            self.pub.publish(self.h_cp)        

        else:
            self.lost_count += 1
            print('Oops(*.*)!! it seems we lost the tag now. Count: {}'.format(self.lost_count))
    
    # def main(self):
        # rospy.Subscriber('/tag_detections_raw', AprilTagDetectionRawArray, self.callback_cam)
        
        # while not self.ctrl_c:

        #     self.h_cp = geometry_msgs.msg.PoseStamped()
        #     self.h_cp.header = Header()
        #     self.h_cp.header.stamp = rospy.Time.now() 
        #     self.h_cp.pose.position.x = self.t_C_P[0]
        #     self.h_cp.pose.position.y = self.t_C_P[1]
        #     self.h_cp.pose.position.z = 1

        #     self.h_cp.pose.orientation.w = 1
        #     self.h_cp.pose.orientation.x = 0
        #     self.h_cp.pose.orientation.y = 0
        #     self.h_cp.pose.orientation.z = 0

        #     self.pub.publish(self.h_cp)

            # tf_W_M = TransformBroadcaster()
            # tf_W_M.sendTransform((self.t_C_P[0], self.t_C_P[1], self.t_C_P[2]),  (0,0,0,1), rospy.Time.now(), 'MMMMMMMMMMMMMM', 'world')        


            # self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('cam_pojected')
    try:
        ci = CameraIntrinsic()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
#! /usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

class Slam:

    def __init__(self):
        self.t_w2sb = np.array([0,0,0])
        self.r_w2sb = np.array([0,0,0,1])

        # ------ publishers and subscribers --------
        self.slam_publisher = rospy.Publisher('/cap_ds/slam_out_pose', PoseStamped, queue_size=10)
        self.slam_subscriber = rospy.Subscriber('/slam_out_pose', PoseStamped,self.callback_slam)


    def callback_slam(self, w2sb):
        self.t_w2sb = np.array([w2sb.pose.position.x, w2sb.pose.position.y, w2sb.pose.position.z])
        self.r_w2sb = np.array([w2sb.pose.orientation.x, w2sb.pose.orientation.y, w2sb.pose.orientation.z, w2sb.pose.orientation.w])

        slam_msg = PoseStamped()
        slam_msg.header.stamp = rospy.Time.now()
        slam_msg.header.frame_id = 'base_link'

        slam_msg.pose.position.x = self.t_w2sb[0]
        slam_msg.pose.position.y = self.t_w2sb[1]
        slam_msg.pose.position.z = self.t_w2sb[2]

        slam_msg.pose.orientation.x = self.r_w2sb[0]
        slam_msg.pose.orientation.y = self.r_w2sb[1]
        slam_msg.pose.orientation.z = self.r_w2sb[2]
        slam_msg.pose.orientation.w = self.r_w2sb[3]
        
        self.slam_publisher.publish(slam_msg)

if __name__ == '__main__':
    rospy.init_node('capds_slam')
    try:
        s = Slam()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

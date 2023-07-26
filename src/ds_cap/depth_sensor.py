#! /usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

class DepthSensor:

    def __init__(self):
        self.depth = np.array([0])

        # ------ publishers and subscribers --------
        self.depth_publisher = rospy.Publisher('/cap_ds/depth', PoseStamped, queue_size=10)
        self.depth_subscriber = rospy.Subscriber('/bluerov/mavros/global_position/rel_alt', Float64, self.callback_depthSensor)


    def callback_depthSensor(self, depth):
        self.depth = -np.array([depth.data])

        self.depth_msg = PoseStamped()
        self.depth_msg.header.stamp = rospy.Time.now()
        self.depth_msg.header.frame_id = 'base_link'

        self.depth_msg.pose.position.x = 0
        self.depth_msg.pose.position.y = 0
        self.depth_msg.pose.position.z = self.depth

        self.depth_msg.pose.orientation.x = 0
        self.depth_msg.pose.orientation.y = 0
        self.depth_msg.pose.orientation.z = 0
        self.depth_msg.pose.orientation.w = 1

        self.depth_publisher.publish(self.depth_msg)

if __name__ == '__main__':
    rospy.init_node('capds_depth')
    try:
        ds = DepthSensor()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

#! /usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import Imu

class ImuSensor:

    def __init__(self):
        self.gyro_data = np.array([[0],[0],[0]])
        self.accel_data = np.array([[0],[0],[0]])


        # ------ publishers and subscribers --------
        self.imu_publisher = rospy.Publisher('/cap_ds/imu', Imu, queue_size=10)
        self.imu_subscriber = rospy.Subscriber('/imu/data', Imu ,self.callback_imu)


    def callback_imu(self, imu):
        self.gyro_data = np.array([[imu.angular_velocity.x],[imu.angular_velocity.y],[imu.angular_velocity.z]])
        self.accel_data = np.array([[imu.linear_acceleration.x],[imu.linear_acceleration.y],[imu.linear_acceleration.z]])

        self.imu_msg = Imu()
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = 'sensor'

        self.imu_msg.angular_velocity.x = self.gyro_data[0]
        self.imu_msg.angular_velocity.y = self.gyro_data[1]
        self.imu_msg.angular_velocity.z = self.gyro_data[2]


        self.imu_msg.linear_acceleration.x = self.accel_data[0]       
        self.imu_msg.linear_acceleration.y = self.accel_data[1]
        self.imu_msg.linear_acceleration.z = self.accel_data[2]
        
        self.imu_publisher.publish(self.imu_msg)

if __name__ == '__main__':
    rospy.init_node('capds_imu')
    try:
        ise = ImuSensor()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

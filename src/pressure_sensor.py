#!/usr/bin/env python3
import tf
import rospy
import math
import tf
import numpy as np
from tf import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import datetime
import time
from std_msgs.msg import Float64
import geometry_msgs.msg

class PressureSensor:

    def __init__(self):

        # Initialize pressure sensor data
        self.pressure_sensor = 0

        # ------ publishers and subscribers --------
        self.pressure_sensor_publisher = rospy.Publisher('/pressure_sensor', Float64, queue_size=1)
        self.pub_one_input = Float64()

         # ------ rate and shutdown ------
        self.rate = rospy.Rate(50.0)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    # ------ publisher for clean shutdown ------
    def publisher_def(self, pub_one_input):
        while not self.ctrl_c:
            self.pressure_sensor_publisher.publish(pub_one_input)
            rospy.loginfo("System is shutting down. Stopping sending information from pressure sensor......")
            break

    # ----- shutdown stuff ---------
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        
        self.stop()
        self.ctrl_c = True
        
    def stop(self):
        # rospy.loginfo("shutdown hook")
        
        # put safe shutdown commands here
        self.pub_one_input.data = 0.0
        self.publisher_def(self.pub_one_input)  # publish zero translation and rotation information when you hit ctrl-c

    def callback_pressure_sensor(self, pressure_sensor):
        self.pressure_sensor = pressure_sensor.data
        
    def main(self):

        rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.callback_pressure_sensor)
        while not self.ctrl_c:

            self.pressure_sensor_publisher.publish(self.pressure_sensor)

            self.rate.sleep()


if __name__ == "__main__":  

    rospy.init_node("pressure_sensor")
    ps = PressureSensor()
    try:
        ps.main()
    except rospy.ROSInterruptException:
        pass

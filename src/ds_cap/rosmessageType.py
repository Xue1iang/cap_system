#! /usr/bin/env python3

from apriltag_ros.msg import AprilTagDetectionRawArray
from sensor_msgs.msg import Imu

message = Imu()
attrs = dir(message.angular_velocity)
for attr in attrs:
    print(attr)
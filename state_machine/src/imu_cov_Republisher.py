#!/usr/bin/env python
# coding: utf-8

######### IMPORT LIBRARIES #########

import rospy
import numpy as np

from sensor_msgs.msg import Imu



class imu_msg_repub():
    def __init__(self):
        self.imu_msg = Imu()
        self.high_covariance = (1e-2, 0.0,  0.0,
                            # put a really high covariance. Value wont't be considered by the filter
                            0.0, 100,   0.0, 
                            0.0, 0.0, 1e-2)
        self.low_covariance = (1e-2, 0.0,  0.0,
                            0.0, 1e-2, 0.0,
                            0.0, 0.0,  1e-2)
        self.publisher = rospy.Publisher('rtabmap/imu_cov', Imu, queue_size=10)
        self.subscriber = rospy.Subscriber('rtabmap/imu', Imu, self.set_covariance)
        
    def set_covariance(self, in_msg):
        self.imu_msg = in_msg
        # change imu covariance value
        omega = in_msg.angular_velocity.y
        if abs(omega) > 0.1:
            self.imu_msg.angular_velocity_covariance = self.low_covariance
        else:
            self.imu_msg.angular_velocity_covariance = self.high_covariance
        self.publisher.publish(self.imu_msg)
           

# ros init
rospy.init_node('imu_covariance_republisher', anonymous=True)

imu_class = imu_msg_repub()


while not rospy.is_shutdown():
    rospy.spin()


#!/usr/bin/env python
# coding: utf-8
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from math import sqrt
import numpy as np

max_vel = rospy.get_param('/joy_to_twist/max_lin_vel', 0.25)
max_ang = rospy.get_param('/joy_to_twist/max_ang_vel', np.pi/5.0)
local_joy= rospy.get_param('/joy_to_twist/local_joy',False)

######### WATCHDOG ON /joy #########
# watchdog max value (cycles)
watchdog_max = 50;
# watchdog counter init
watchdog_cnt = 0;

def callback(data):
    global max_vel, max_ang, watchdog_cnt, watchdog_max
    if local_joy:
        x = max_vel * data.axes[1]
        theta = max_ang * data.axes[2]
    else:
        x = max_vel * data.axes[1]
        theta = max_ang * data.axes[3]
    twist = Twist()
    twist.linear.x = x
    twist.angular.z = theta
    watchdog_cnt = watchdog_max
    pub.publish(twist)

rospy.init_node('joy_node', anonymous=True)
rospy.Subscriber("joy", Joy, callback)
pub = rospy.Publisher('cmd_vel_joy', Twist, queue_size=10)

r = rospy.Rate(100)
while not rospy.is_shutdown():
    if watchdog_cnt>0:
        watchdog_cnt-=1
    else:
        pub.publish(Twist())
    r.sleep()

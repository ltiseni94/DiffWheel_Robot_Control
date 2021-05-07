#!/usr/bin/env python
# coding: utf-8

######### IMPORT LIBRARIES #########

import rospy
import socket
import math
import numpy as np
import tf2_ros
import tf
import tf_conversions
from geometry_msgs.msg import Twist, TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from kinematics_matrix import inv_jacobian, encoder_constant

######### DEFINE "GLOBAL" VARIABLES AND PARAMETERS #########

ROBOT_LOCAL = rospy.get_param('/udp_receiver/robot_local', False)
wh_speeds_enc = np.zeros(2, np.float32)
input_mask = encoder_constant * np.array([-1.0, 1.0], np.float32)
v_rel = np.array([0.0, 0.0], np.float32)   # [m/s], [m/s], [1/s]
RATE = 100.0    # [Hz]
dt = 1.0/RATE   # [s]
pub_rate = 20.0
N = (int)(RATE/pub_rate)

######### CUSTOM CLASSES #########

class RobotOdom():
    def __init__(self, rate=RATE, robot_local=ROBOT_LOCAL):
        self.x=0
        self.y=0
        self.th=0
        self.dt=1.0/rate
        self.robot_local=robot_local
        self.publisher = rospy.Publisher('odom_wheels', Odometry, queue_size=10)
        self.tfbroadcaster = tf.TransformBroadcaster() # create a tf
        self.twist_local = TwistWithCovariance()
        self.twist_local.covariance[0]=1e-6
        self.twist_local.covariance[7]=1e-6
        self.twist_local.covariance[14]=1e-6
        self.twist_local.covariance[21]=1e-6
        self.twist_local.covariance[28]=1e-6
        self.twist_local.covariance[35]=2e-3 # the imu one is 1e-2
        self.odom_msg = Odometry()
        
    def update_twist(self, speed_array):
        self.twist_local.twist.linear.x=speed_array[0]
        self.twist_local.twist.angular.z=speed_array[1]
        
    def update_odom(self):
        dx = self.twist_local.twist.linear.x*(math.cos(self.th))*self.dt
        dy = self.twist_local.twist.linear.x*(math.sin(self.th))*self.dt
        dth = self.twist_local.twist.angular.z*self.dt
        self.x += dx
        self.y += dy
        self.th += dth
        print(self.th)
        
    def publish_odom_msg(self):
        now=rospy.Time.now()
        self.odom_msg.header.stamp=now
        self.odom_msg.header.frame_id="odom"
        self.odom_msg.child_frame_id="base_link"
        self.odom_msg.pose.pose.position.x=self.x
        self.odom_msg.pose.pose.position.y=self.y
        q=tf_conversions.transformations.quaternion_from_euler(0,0,self.th)
        self.odom_msg.pose.pose.orientation.x=q[0]
        self.odom_msg.pose.pose.orientation.y=q[1]
        self.odom_msg.pose.pose.orientation.z=q[2]
        self.odom_msg.pose.pose.orientation.w=q[3]
        self.odom_msg.pose.covariance[0]=1e-4
        self.odom_msg.pose.covariance[7]=1e-4
        self.odom_msg.pose.covariance[35]=1e-1
        self.odom_msg.twist=self.twist_local
        
        self.publisher.publish(self.odom_msg)
                
        if not (self.robot_local):
            self.tfbroadcaster.sendTransform([self.x, self.y, 0.0], # load position
                    q,                  # load quaternion
                    now,   # send current time
                    "base_link",        # to
                    "odom")             # from
                
    
######### SET LOCAL SOCKET IP ADDRESS AND UDP PORT #########
personal_IP = "192.168.0.100"
personal_port = 11111
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((personal_IP, personal_port))

# ros init
rospy.init_node('udp_receiver', anonymous=True)
r = rospy.Rate(RATE)

robot_odom = RobotOdom()
i=0

while not rospy.is_shutdown():
    data, _ = sock.recvfrom(10)
    # interpret first 8 bytes as wheel velocities
    wh_speeds_enc = np.frombuffer(data, dtype=np.float32, count=2)*input_mask
    # interpret last 2 bytes as robot state
    enable_input = np.frombuffer(data, dtype=np.uint8, count=2, offset=8)
    # calculate linear and angular relative speed of the robot
    v_rel = np.matmul(inv_jacobian, wh_speeds_enc)
    robot_odom.update_twist(v_rel)
    robot_odom.update_odom()
    
    # if i == N:
    #     robot_odom.publish_odom_msg()
    #     i = 0
    # i+=1
    
    robot_odom.publish_odom_msg()
    
    #r.sleep()


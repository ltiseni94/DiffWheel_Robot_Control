#!/usr/bin/env python
# coding: utf-8

######### IMPORT LIBRARIES #########

import rospy
import socket
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped
from kinematics_matrix import inv_jacobian, encoder_constant

######### DEFINE "GLOBAL" VARIABLES AND PARAMETERS #########

#init encoder raw count value
wh_speeds_enc = np.zeros(2, np.float32)
#encoder input mask
input_mask = encoder_constant * np.array([-1.0, 1.0], np.float32)
# relative velocity and angular speed
v_rel = np.array([0.0, 0.0], np.float32)   # [m/s], [m/s], [1/s]
# odometry values
# odom = np.array([0.0, 0.0, 0.0], np.float32)    # [m], [m], [rad]
# old_odom = np.array([0.0, 0.0, 0.0], np.float32)    # [m], [m], [rad]
# node rate and time step
RATE = 100.0    # [Hz]
dt = 1.0/RATE   # [s]
# reset boolean: if 0 update current odometry, if 1 reset x, y, teta to 0
# RESET_FLAG = False

######### CUSTOM FUNCTIONS #########

# broadcaster of the odometry ad tf message
# def handle_robot_pose(_odom):
#    br = tf.TransformBroadcaster()  # create a tf
#    br.sendTransform([_odom[0], _odom[1], 0.0], # load position
#                     tf.transformations.quaternion_from_euler(0, 0, _odom[2]),  # load quaternion
#                     rospy.Time.now(),  # send corrent time
#                     "base_link",    # to
#                     "odom")         # from

#def reset_robot_pose(req):
#    if req.reset == 1:
#        global odom, old_odom
#       # save the odometry before the reset
#        old_odom = odom
#        # reset odometry
#        odom = np.array([0.0, 0.0, 0.0], np.float32)  # [m], [m], [rad]
#        return reset_odomResponse(True)

######### SET LOCAL SOCKET IP ADDRESS AND UDP PORT #########
personal_IP = "192.168.0.100"
personal_port = 11111
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((personal_IP, personal_port))

# ros init
rospy.init_node('udp_receiver', anonymous=True)
pub1 = rospy.Publisher('odom_wheels', Twist, queue_size=10)
pub2 = rospy.Publisher('twist_n_covariance_wheels', TwistWithCovarianceStamped, queue_size=10)
twist=Twist()
tw_cov = TwistWithCovarianceStamped()
# ros set rate
r = rospy.Rate(RATE)

print_counter = 0
print_counter2 = 0

# create the service
# ser = rospy.Service('reset_msg', reset_odom, reset_robot_pose)

while not rospy.is_shutdown():
    # RECEIVE WHEEL VELOCITY CODE
    # Receive incoming packet
    data, _ = sock.recvfrom(10)   #receive 18 bytes
    # interpret first 16 bytes as wheel velocities
    wh_speeds_enc = np.frombuffer(data, dtype=np.float32, count=2)*input_mask
    # interpret last 2 bytes as robot state
    enable_input = np.frombuffer(data, dtype=np.uint8, count=2, offset=8)
    '''
        # Print values on terminal
        print_counter += 1
        if print_counter > 100:
            print(wh_speeds_enc * 30.0 / np.pi)
            print(enable_input)
            print_counter = 0
    '''
    #   CALCULATE ODOMETRY
    # calculate linear and angular relative speed of the robot
    v_rel = np.matmul(inv_jacobian, wh_speeds_enc)
    twist.linear.x=v_rel[0]
    twist.angular.z=v_rel[1]   
    pub1.publish(twist)
    
    ### create twist message with stamp and covariance for robot loclization ###
    tw_cov.header.stamp = rospy.Time.now()
    tw_cov.header.frame_id = "odom"
    # tw_cov.header.child_frame_id = "base_link"
    
    tw_cov.twist.twist.linear.x=v_rel[0]
    tw_cov.twist.twist.angular.z=v_rel[1] 
    tw_cov.twist.covariance[0] = 1e-6
    tw_cov.twist.covariance[35] = 1e-6
    
    pub2.publish(tw_cov)
    
    # update odom_positions
    # odom += v_rel * dt
    
    #print_counter2 += 1
    #if print_counter2 > 100:
    #    print("vel = ", v_rel)
    #    print("odom = ", odom * np.array([1.0, 180.0/np.pi]))
    #   print_counter2 = 0
    
    # publish tf message
    # handle_robot_pose(odom)
    # graph()
    # ROS SLEEP
    r.sleep()


#! /usr/bin/env python3

import rospy
from time import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from math import *
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler


#Gain for Simulator
K_RO = 2
K_ALPHA = 27.0
K_BETA = -8.0
V_CONST = 0.3 # [m/s]

#Gain for REAL ROBOT
'''
K_RO = 2
K_ALPHA = 9
K_BETA = -1.8
V_CONST = 0.35
'''

GOAL_DIST_THRESHOLD = 0.1
GOAL_ANGLE_THRESHOLD = 25

def getRotation(odomMsg):
    orientation_q = odomMsg.pose.pose.orientation
    orientation_list =  [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    return yaw

def getPosition(odomMsg):
    x = odomMsg.pose.pose.position.x
    y = odomMsg.pose.pose.position.y
    return (x , y)

def get_linear_vel(odomMsg):
    return odomMsg.twist.twist.linear.x

def get_angular_vel(odomMsg):
    return odomMsg.twist.twist.angular.z

def create_vel_msg(v, w):
    velMsg = Twist()
    velMsg.linear.x = v
    velMsg.linear.y = 0
    velMsg.linear.z = 0
    velMsg.angular.x = 0
    velMsg.angular.y = 0
    velMsg.angular.z = w
    return velMsg

def robotStop(velPub):
    velMsg = create_vel_msg(0.0, 0.0)
    return velPub.publish(velMsg)

def robotSetPos(setPosPub, x, y, theta):
    checkpoint = ModelState()
    checkpoint.model_name = 'RaspberryPiMouse'

    checkpoint.pose.position.x = x
    checkpoint.pose.position.y = y
    checkpoint.pose.position.z = 0.0

    #convert theta to orientation
    [x_q,y_q,z_q,w_q] = quaternion_from_euler(0.0,0.0,radians(theta))

    checkpoint.pose.orientation.x = x_q
    checkpoint.pose.orientation.y = y_q
    checkpoint.pose.orientation.z = z_q
    checkpoint.pose.orientation.w = w_q
    
    checkpoint.twist.linear.x = 0.0
    checkpoint.twist.linear.y = 0.0
    checkpoint.twist.linear.z = 0.0

    checkpoint.twist.angular.x = 0.0
    checkpoint.twist.angular.y = 0.0
    checkpoint.twist.angular.z = 0.0

    setPosPub.publish(checkpoint)
    return ( x , y , theta )

def robotSetRandomPos(setPosPub):
    x_range = np.array([-0.4, 0.6, 0.6, -1.4, -1.4, 2.0, 2.0, -2.5, 1.0, -1.0])
    y_range = np.array([-0.4, 0.6, -1.4, 0.6, -1.4, 1.0, -1.0, 0.0, 2.0, 2.0])
    theta_range = np.arange(0, 360, 15)
    #theta_range = np.array([0, 30, 45, 60, 75, 90])

    ind = np.random.randint(0,len(x_range))
    ind_theta = np.random.randint(0,len(theta_range))

    '''
    x = x_range[ind]
    y = y_range[ind]
    theta = theta_range[ind_theta]
    '''

    #start at the center
    x = 0.0
    y = 0.0
    theta = 0

    checkpoint = ModelState()

    checkpoint.model_name = 'RaspberryPiMouse'

    checkpoint.pose.position.x = x
    checkpoint.pose.position.y = y
    checkpoint.pose.position.z = 0.0

    [x_q,y_q,z_q,w_q] = quaternion_from_euler(0.0,0.0,radians(theta))

    checkpoint.pose.orientation.x = x_q
    checkpoint.pose.orientation.y = y_q
    checkpoint.pose.orientation.z = z_q
    checkpoint.pose.orientation.w = w_q

    checkpoint.twist.linear.x = 0.0
    checkpoint.twist.linear.y = 0.0
    checkpoint.twist.linear.z = 0.0

    checkpoint.twist.angular.x = 0.0
    checkpoint.twist.angular.y = 0.0
    checkpoint.twist.angular.z = 0.0

    setPosPub.publish(checkpoint)
    return ( x , y , theta )

def robotFeedbackControl(velPub, x, y, theta, x_goal, y_goal, theta_goal):
    if theta_goal >= pi:
        theta_goal_norm = theta_goal - 2 * pi
    else:
        theta_goal_norm = theta_goal

    ro = sqrt(pow((x_goal - x), 2) + pow((y_goal - y), 2))
    lamda = atan2( y_goal - y , x_goal - x)

    alpha = (lamda - theta + pi) % (2 * pi) - pi #-360degrees
    beta = (theta_goal - lamda + pi) % (2 * pi) - pi

    if ro < GOAL_DIST_THRESHOLD and degrees(abs(theta-theta_goal_norm)) < GOAL_ANGLE_THRESHOLD:
        status = 'Goal Position reached'
        print ('Goal Position reached')
        v = 0
        w = 0
        v_scal = 0
        w_scal = 0
    else:
        status = 'Goal position not reached'
        v = K_RO * ro
        w = K_ALPHA * alpha + K_BETA * beta
        v_scal = v / abs(v) * V_CONST
        w_scal = w / abs(v) * V_CONST

    velMsg = create_vel_msg(v_scal, w_scal)
    velPub.publish(velMsg)

    return status

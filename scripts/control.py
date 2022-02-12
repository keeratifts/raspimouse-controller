#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""

@author: keykeerati
@date: 2021/12/06

"""

from numpy.random.mtrand import random
import rospy
import collections
from ar_track_alvar_msgs.msg import AlvarMarkers
from time import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from math import *
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import random


#Gain for Simulator
# K_RO = 3
# K_ALPHA = 5.0
# K_BETA = -8.0
# V_CONST = 0.2 # [m/s]
# W_CONST = 0.2
# GOAL_DIST_THRESHOLD = 0.3

#Gain for REAL ROBOT

K_RO = 2
K_ALPHA = -3.0
K_BETA = 3
V_CONST = 0.12
W_CONST = 0.3


GOAL_DIST_THRESHOLD = 0.09 
GOAL_ANGLE_THRESHOLD = 35

global roll, pitch, yaw


def get_robot_location(AlvarMsg, robot):

    for m in AlvarMsg.markers:
        marker_id = m.id    

        if marker_id <= 5:
            marker_pose = m.pose.pose
            pos = marker_pose.position
            ori = marker_pose.orientation
            ori_list = [ori.y, ori.z, ori.x, ori.w]

            #transform orientation to euler
            (roll, pitch, yaw) = euler_from_quaternion(ori_list)
            robot[marker_id] = {'x': pos.y, 'y': pos.z, 'yaw': yaw}
            robot = collections.OrderedDict(sorted(robot.items())) #sort dict by markers no.

    return robot

def get_goal_location(AlvarMsg, goal):

    for m in AlvarMsg.markers:
        marker_id = m.id
        if marker_id >= 20:
            marker_pose = m.pose.pose
            pos = marker_pose.position

            goal[marker_id] = {'x': pos.y, 'y': pos.z}
            goal = collections.OrderedDict(sorted(goal.items())) #sort dict by markers no.
    
    return goal

def choose_goal(goal, robot):
    all_d = np.empty((0, len(robot)), float)
    for i in range(len(goal)):
        distance = []
        for j in range(100):
            if j in robot:
                distance.append(sqrt(pow((goal[i]['x'][0] - robot[j]['x']), 2) + pow((goal[i]['y'][0] - robot[j]['y']), 2)))
        all_d = np.append(all_d, [distance], axis = 0)

    while not np.isnan(all_d).all():
        result = np.where(all_d == np.nanmin(all_d))
        listofCordinates = list(zip(result[0], result[1]))
        #print (listofCordinates)
        for cord in listofCordinates:
            k = {'goal': goal[cord[0]]}
            robot[cord[1]].update(k)
            all_d[cord[0]] = np.NaN
            all_d[:,cord[1]] = np.NaN

    return robot

def closest_distance(x, y, robot):
    all_d = np.empty((0, len(robot)), float)

    for i in range(len(x)):
        distance = []
        for j in range(100):   
            if j in robot:
                distance.append(sqrt(pow((x[i] - robot[j]['x']), 2) + pow((y[i] - robot[j]['y']), 2)))
        all_d = np.append(all_d, [distance], axis = 0)
    
    while not np.isnan(all_d).all():
        result = np.where(all_d == np.nanmin(all_d))
        listofCordinates = list(zip(result[0], result[1]))
        for cord in listofCordinates:
            #print ('goal: %d, robot: %d' % (cord[0], cord[1] + 1))
            k = {'GOAL_X': x[cord[0]], 'GOAL_Y': y[cord[0]]}
            robot[cord[1]].update(k)
            all_d[cord[0]] = np.NaN
            all_d[:,cord[1]] = np.NaN
    
    return robot

def set_goal_dict(log_file):
    dict = {}
    for i in range(len(log_file)):
        goal, x, y = ([], [], [])
        for row in csv.reader(log_file[i], delimiter = ','):
            goal.append(row)
        for j in range(len(goal)):
            x.append(round(float(goal[j][0]), 4))
            y.append(round(float(goal[j][1]), 4))
        dict[i] = {'x': x, 'y': y}
    
    return dict

def choose_goal(goal, robot):
    all_d = np.empty((0, len(robot)), float)
    for i in range(len(goal)):
        distance = []
        for j in range(100):
            if j in robot:
                distance.append(sqrt(pow((goal[i]['x'][0] - robot[j]['x']), 2) + pow((goal[i]['y'][0] - robot[j]['y']), 2)))
        all_d = np.append(all_d, [distance], axis = 0)

    while not np.isnan(all_d).all():
        result = np.where(all_d == np.nanmin(all_d))
        listofCordinates = list(zip(result[0], result[1]))
        #print (listofCordinates)
        for cord in listofCordinates:
            k = {'goal': goal[cord[0]]}
            robot[cord[1]].update(k)
            all_d[cord[0]] = np.NaN
            all_d[:,cord[1]] = np.NaN

    return robot

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
    checkpoint.model_name = 'raspi_0'

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

def robotSetRandomPos(setPosPub, model):
    x_range = np.array([-0.4, 0.6, 0.6, -1.4, -1.4, 2.0, 2.0, -2.5, 1.0, -1.0])
    y_range = np.array([-0.4, 0.6, -1.4, 0.6, -1.4, 1.0, -1.0, 0.0, 2.0, 2.0])
    theta_range = np.arange(0, 360, 15)
    #theta_range = np.array([0, 30, 45, 60, 75, 90])

    ind = np.random.randint(0,len(x_range))
    ind_theta = np.random.randint(0,len(theta_range))

    x = round(random.uniform(-4, 4), 2)
    y = round(random.uniform(-4, 4), 2)
    theta = round(random.uniform(0, 360), 2)

    #start at the center

    checkpoint = ModelState()

    checkpoint.model_name = model

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

def robotPosfromcoords(setPosPub, coords, N, D):
    data = np.genfromtxt(coords, delimiter=',')
    for i in range(0, N):
        (x, y) = (data[i][0]/D, data[i][1]/D)
        theta = round(random.uniform(0, 360), 2)

    #start at the center

        checkpoint = ModelState()

        checkpoint.model_name = 'raspi_'+str(i)

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
    if ro < GOAL_DIST_THRESHOLD:
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

def robotFeedbackControl_without_beta(velPub, x, y, theta, x_goal, y_goal):

    ro = sqrt(pow((x_goal - x), 2) + pow((y_goal - y), 2))
    lamda = atan2( y_goal - y , x_goal - x)

    alpha = (lamda - theta + pi) % (2 * pi) - pi #-360degrees
    
    if ro <= GOAL_DIST_THRESHOLD:
        status = 'Goal Position reached'
        #print ('Goal Position reached')
        v = 0
        w = 0
        v_scal = 0
        w_scal = 0
    else:
        status = 'Goal position not reached'
        v = K_RO * ro
        w = K_ALPHA * alpha
        if v >= V_CONST:
            v_scal = v / abs(v) * V_CONST
        else:
            v_scal = K_RO * ro

        w_scal = w / abs(v) * W_CONST
        # if w_scal >= 3:
        #      w_scal = 1.0
        # elif w_scal <= -3:
        #      w_scal = -1.0

    velMsg = create_vel_msg(v_scal, w_scal)
    velPub.publish(velMsg)

    return status

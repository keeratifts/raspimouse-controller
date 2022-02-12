#! /usr/bin/env python
#run this script for real robot

import rospy
from time import time
from time import sleep
from datetime import datetime

from control import *


X_INIT = 0.0
Y_INIT = 0.0
THETA_INIT = 0.0
X_GOAL = -0.0
Y_GOAL = 0.0
THETA_GOAL = 180.0



if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        rate = rospy.Rate(10)
        velPub_2 = rospy.Publisher('raspi_2/cmd_vel', Twist, queue_size=10)
        velPub_1 = rospy.Publisher('raspi_1/cmd_vel', Twist, queue_size=10)
        velPub_0 = rospy.Publisher('raspi_0/cmd_vel', Twist, queue_size=10)

        robot_in_pos = False
        raspi = {}
        goal = {}
        i = 0
        sleep(1)

        while not rospy.is_shutdown():
            if not robot_in_pos:
                robotStop(velPub_0)
                robotStop(velPub_1)
                robotStop(velPub_2)
                AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
                raspi = get_robot_location(AlvarMsg, raspi)
                goal = get_goal_location(AlvarMsg, goal)
                if len(raspi) == 3:
                    
                    ### USE MARKERS AS GOALS ###
                    X_GOAL = [goal[22]['x'], goal[25]['x'], goal[24]['x']]
                    Y_GOAL = [goal[22]['y'], goal[25]['y'], goal[24]['y']]

                    robot_in_pos = True
                    print('\r\nInitial position:')
                    print('Raspi_0: (x, y, theta) = (%.2f, %.2f, %.2f)' % (raspi[0]['x'], raspi[0]['y'], degrees(raspi[0]['yaw'])))
                    print('Raspi_1: (x, y, theta) = (%.2f, %.2f, %.2f)' % (raspi[1]['x'], raspi[1]['y'], degrees(raspi[1]['yaw'])))
                    print('Raspi_2: (x, y, theta) = (%.2f, %.2f, %.2f)' % (raspi[2]['x'], raspi[2]['y'], degrees(raspi[2]['yaw'])))
                    print('')

                else:
                    robot_in_pos = False

            else:
                # AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
                # raspi = get_robot_location(AlvarMsg, raspi)
                # print (raspi)
                # status = robotFeedbackControl_without_beta(velPub, raspi[2]['x'], raspi[2]['y'], raspi[2]['yaw'], X_GOAL, Y_GOAL)
                # if status == 'Goal Position reached':
                #     robotStop(velPub)
                #     rospy.signal_shutdown('End of testing')
                
                ### USE MARKERS AS GOALS ###
                AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
                raspi = get_robot_location(AlvarMsg, raspi)
                status_2 = robotFeedbackControl_without_beta(velPub_2, raspi[2]['x'], raspi[2]['y'], raspi[2]['yaw'], X_GOAL[2], Y_GOAL[2])
                status_1 = robotFeedbackControl_without_beta(velPub_1, raspi[1]['x'], raspi[1]['y'], raspi[1]['yaw'], X_GOAL[1], Y_GOAL[1])
                status_0 = robotFeedbackControl_without_beta(velPub_0, raspi[0]['x'], raspi[0]['y'], raspi[0]['yaw'], X_GOAL[0], Y_GOAL[0])
                if status_0 == 'Goal Position reached' and status_1 == 'Goal Position reached' and status_2 == 'Goal Position reached':
                    robotStop(velPub_0)
                    robotStop(velPub_1)
                    robotStop(velPub_2)
                    rospy.signal_shutdown('End of testing')

                
                # if status == 'Goal Position reached':
                #     i += 1
                #     print ('%d goal reached' % (i))
                #     if i == len(X_GOAL):
                #         robotStop(velPub)
                #         rospy.signal_shutdown('End of testing')

    except rospy.ROSInterruptException:
        robotStop(velPub_0)
        robotStop(velPub_1)
        robotStop(velPub_2)
        print('Terminated')
        pass
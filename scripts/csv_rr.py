#! /usr/bin/env python
#run this script for real robot

import rospy
from time import time
from time import sleep
from datetime import datetime
from control import *

N = 4
goal = dict()

for i in range(0, N):
    goal[i] = open('/home/robolab/Coverage control/Trajectory_Log/goal_'+str(i+1)+'.csv','r')

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        rate = rospy.Rate(10)
        (velPub, goalx_r, goaly_r) = (dict(), dict(), dict())
        for i in range(0, N):
            velPub[i] = rospy.Publisher('raspi_'+str(i)+'/cmd_vel', Twist, queue_size=10)
        
        goal = set_goal_dict(goal)
        robot_in_pos = False
        raspi = {}
        j = 0
        sleep(1)

        while not rospy.is_shutdown():
            if not robot_in_pos:
                for i in range(0, N):
                    robotStop(velPub[i])

                AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
                raspi = get_robot_location(AlvarMsg, raspi)
                if len(raspi) == N:
                    raspi = choose_goal(goal, raspi)
                    robot_in_pos = True
                    print('\r\nInitial position:')
                    for i in range(0, N):
                        print('Raspi_'+str(i)+': (x, y, theta) = (%.2f, %.2f, %.2f)' % (raspi[i]['x'], raspi[i]['y'], degrees(raspi[i]['yaw'])))
                        goalx_r[i] = raspi[i]['goal']['x']
                        goaly_r[i] = raspi[i]['goal']['y']
                    print('')

                else:
                    robot_in_pos = False

            else:
                if j >= len(goalx_r[0]):
                    if all(value=='Goal Position reached' for value in status.values()):
                        for i in range(0, N):
                            robotStop(velPub[i])
                        print ("Mission completed")    
                        rospy.signal_shutdown('End of testing')
                    else:
                        for i in range(0, N):
                            raspi = get_robot_location(AlvarMsg, raspi)
                            if len(raspi) == N:
                                for i in range(0, N):
                                    status[i] = robotFeedbackControl_without_beta(velPub[i], raspi[i]['x'], raspi[i]['y'], raspi[i]['yaw'], goalx_r[i][len(goalx_r[i])-1], goaly_r[i][len(goalx_r[i])-1])
                else:
                    status = {}
                    AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
                    raspi = get_robot_location(AlvarMsg, raspi)
                    if len(raspi) == N:
                        for i in range(0, N):
                            status[i] = robotFeedbackControl_without_beta(velPub[i], raspi[i]['x'], raspi[i]['y'], raspi[i]['yaw'], goalx_r[i][j], goaly_r[i][j])
                            if j == 0:
                                if all(value=='Goal Position reached' for value in status.values()):
                                    j += 1
                                else:
                                    j = 0
                            else:
                                j += 1
                        # if all(value=='Goal Position reached' for value in status.values()):
                        #     j += 10
                        #     print (j)


    except rospy.ROSInterruptException:
        for i in range(0, N):
            robotStop(velPub[i])
        print('Terminated')
        pass
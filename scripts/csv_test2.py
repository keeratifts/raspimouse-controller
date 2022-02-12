#! /usr/bin/env python
#run this script for real robot

import rospy
from time import time
from time import sleep
from datetime import datetime
from control import *
from numpy import genfromtxt

N = 6
D = 160
goal = dict()

for i in range(0, N):
    goal[i] = genfromtxt('/home/robolab/Coverage control/Trajectory_Log/goal_'+str(i+1)+'.csv', delimiter=',')

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        rate = rospy.Rate(10)
        (velPub, goalx_r, goaly_r) = (dict(), dict(), dict())
        for i in range(0, N):
            velPub[i] = rospy.Publisher('raspi_'+str(i)+'/cmd_vel', Twist, queue_size=10)
        
        robot_in_pos = False
        raspi = {}
        j = 0
        sleep(1)

        while not rospy.is_shutdown():
            AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
            raspi = get_robot_location(AlvarMsg, raspi)

            if not robot_in_pos:
                for i in range(0, N):
                    robotStop(velPub[i])

                AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
                raspi = get_robot_location(AlvarMsg, raspi)
                if len(raspi) == N:
                    robot_in_pos = True
                    print('\r\nInitial position:')
                    for i in range(0, N):
                        print('Raspi_'+str(i)+': (x, y, theta) = (%.2f, %.2f, %.2f)' % (raspi[i]['x'], raspi[i]['y'], degrees(raspi[i]['yaw'])))
                    print('')

                else:
                    robot_in_pos = False

            else:
                if j >= len(goal[0]):
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
                                    status[i] = robotFeedbackControl_without_beta(velPub[i], raspi[i]['x'], raspi[i]['y'], raspi[i]['yaw'], goal[i][499][0]/D, goal[i][499][1]/D)
                else:
                    status = {}
                    if len(raspi) == N:
                        for i in range(0, N):
                            status[i] = robotFeedbackControl_without_beta(velPub[i], raspi[i]['x'], raspi[i]['y'], raspi[i]['yaw'], goal[i][j][0] / D, goal[i][j][1] / D)
                        if j == 0:
                            if all(value=='Goal Position reached' for value in status.values()):
                                j += 5
                            else:
                                j = 0
                        else:
                            j += 5
                        # if all(value=='Goal Position reached' for value in status.values()):
                        #     j += 10
                        #     print (j)


    except rospy.ROSInterruptException:
        for i in range(0, N):
            robotStop(velPub[i])
        print('Terminated')
        pass
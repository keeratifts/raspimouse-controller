#! /usr/bin/env python
#run this script for real robot

from csv import DictReader
from logging import setLoggerClass
import rospy
from time import time
from time import sleep
from datetime import datetime
from control import *

N = 20
goal = dict()
D = 50

for i in range(0, N):
    goal[i] = open('/home/robolab/Coverage control/Trajectory_Log/goal_'+str(i+1)+'.csv','r')

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        rate = rospy.Rate(10)
        (velPub, odomMsg) = (dict(), dict())
        status = dict()
        for i in range(0, N):
            velPub[i] = rospy.Publisher('raspi_'+str(i)+'/cmd_vel', Twist, queue_size=10)

        goal = set_goal_dict(goal)

        robot_in_pos = False
        raspi = {}
        j = 0
        c = 0
        sleep(1)

        while not rospy.is_shutdown():
            
            for i in range(0, N):
                odomMsg[i] = rospy.wait_for_message('/raspi_'+str(i)+'/odom', Odometry)

            if not robot_in_pos:
                for i in range(0, N):
                    robotStop(velPub[i])
                (x, y, theta) = (dict(), dict(), dict())
                for i in range(0, N):
                    (x[i], y[i]) = getPosition(odomMsg[i])
                    theta[i] = getRotation(odomMsg[i])
                    raspi[i] = {'x': x[i], 'y': y[i], 'yaw': radians(theta[i])}
                
                raspi = choose_goal(goal, raspi)
                robot_in_pos = True

                (goalx_r, goaly_r) = (dict(), dict())
                print('\r\nInitial position:')
                for i in range(0, N):
                    print('Raspi_'+str(i)+': (x, y, theta) = ( %.2f, %.2f, %.2f)' % (x[i], x[i], theta[i]))
                    goalx_r[i] = raspi[i]['goal']['x']
                    goaly_r[i] = raspi[i]['goal']['y']
                print('')

            else:
                if j >= len(goalx_r[1]):
                    if all(value=='Goal Position reached' for value in status.values()):
                        for i in range(0, N):
                            robotStop(velPub[i])
                        print ("Mission completed")
                        rospy.signal_shutdown('End of testing')
                    else:
                        for i in range(0, N):
                            (x[i], y[i]) = getPosition(odomMsg[i])
                            theta[i] = getRotation(odomMsg[i])
                            status[i] = robotFeedbackControl_without_beta(velPub[i], x[i], y[i], theta[i], goalx_r[i][len(goalx_r[i])-1 / 40], goaly_r[i][len(goalx_r[i])-1] / 40)
                            c +=1
                            print (c)

                else:

                    for i in range(0, N):
                        (x[i], y[i]) = getPosition(odomMsg[i])
                        theta[i] = getRotation(odomMsg[i])
                        status[i] = robotFeedbackControl_without_beta(velPub[i], x[i], y[i], theta[i], goalx_r[i][j] / 40, goaly_r[i][j] / 40)

                    # if all(value=='Goal Position reached' for value in status.values()):
                    #     j+=10
                    # else:
                    #     print (j)
                    if j == 0:
                        if all(value=='Goal Position reached' for value in status.values()):
                            j += 5
                        else:
                            j = 0
                    else:
                        j += 5
    except rospy.ROSInterruptException:
        for i in range(0, N):
            robotStop(velPub[i])
        print('Terminated')
        pass
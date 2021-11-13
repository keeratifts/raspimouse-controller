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
X_GOAL = 0.0
Y_GOAL = 0.0
THETA_GOAL = 180.0



if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        rate = rospy.Rate(10)
        velPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        robot_in_pos = False

        sleep(1)

        while not rospy.is_shutdown():
            odomMsg = rospy.wait_for_message('/odom', Odometry)
            if not robot_in_pos:
                robotStop(velPub)
                (x_init, y_init, theta_init) = (0, 0, 0)
                
                #check
                odomMsg = rospy.wait_for_message('/odom', Odometry)
                (x, y) = getPosition(odomMsg)
                theta = degrees(getRotation(odomMsg))
                robot_in_pos = True
                print('\r\nInitial position:')
                print('x = %.2f [m]' % x)
                print('y = %.2f [m]' % y)
                print('theta = %.2f [degrees]' % theta)
                print('')
            else:
                ( x , y ) = getPosition(odomMsg)
                theta = getRotation(odomMsg)

                status = robotFeedbackControl(velPub, x, y, theta, X_GOAL, Y_GOAL, radians(THETA_GOAL))
                if status == 'Goal Position reached':
                    robotStop(velPub)
                    rospy.signal_shutdown('End of testing')

    except rospy.ROSInterruptException:
        robotStop(velPub)
        print('Terminated')
        pass
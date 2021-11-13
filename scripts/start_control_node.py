#! /usr/bin/env python
#run this script for simulator

import rospy
from time import time
from time import sleep
from datetime import datetime

from control import *
import sys
DATA_PATH = '/home/robolab/raspi_ws/src/raspi_control/Data'

GOAL_POSITIONS_X = [ 2.0, -8.0, 0.5, -4, 4]
GOAL_POSITIONS_Y = [ 1.0, 7.0, -1.9, -3, 3,]
GOAL_POSITIONS_THETA = [ 25.0, -40.0, -40, 60, 0,]

PATH_IND = 1

X_GOAL = GOAL_POSITIONS_X[PATH_IND]
Y_GOAL = GOAL_POSITIONS_Y[PATH_IND]
THETA_GOAL = GOAL_POSITIONS_THETA[PATH_IND]

X_traj = np.array([])
Y_traj = np.array([])
THETA_traj = np.array([])
X_goal = np.array([])
Y_goal = np.array([])
THETA_goal = np.array([])

#LOG_DIR = DATA_PATH + '/Log_feedback_1'

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        rate = rospy.Rate(10)

        setPosPub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        velPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        ''''
        #Plot feedback control graph
        log_sim_params = open(LOG_DIR+'/LogSimParams.txt','w+')

        text = 'Simulation parameters: \r\n'
        text = text + 'k_rho = %.3f \r\n' % K_RO
        text = text + 'k_alpha = %.3f \r\n' % K_ALPHA
        text = text + 'k_beta = %.3f \r\n' % K_BETA
        text = text + 'v_const = %.3f \r\n' % V_CONST
        log_sim_params.write(text)
        log_sim_params.close()
        '''

        robot_in_pos = False

        sleep(1)

        while not rospy.is_shutdown():
            odomMsg = rospy.wait_for_message('/odom', Odometry)
            if not robot_in_pos:
                robotStop(velPub)
                (x_init, y_init, theta_init) = robotSetRandomPos(setPosPub)
                
                #check
                odomMsg = rospy.wait_for_message('/odom', Odometry)
                (x, y) = getPosition(odomMsg)
                theta = degrees(getRotation(odomMsg))
                print(theta, theta_init)
                if abs(x-x_init) < 0.05 and abs(y-y_init) < 0.05 and abs(theta - theta_init) < 2:
                    robot_in_pos = True
                    print('\r\nInitial position:')
                    print('x = %.2f [m]' % x)
                    print('y = %.2f [m]' % y)
                    print('theta = %.2f [degrees]' % theta)
                    print('')
                    sleep(1)
                else:
                    robot_in_pos = False
            else:
                ( x , y ) = getPosition(odomMsg)
                theta = getRotation(odomMsg)

                X_traj = np.append(X_traj, x)
                Y_traj = np.append(Y_traj, y)
                THETA_traj = np.append(THETA_traj, degrees(theta))
                X_goal = np.append(X_goal, X_GOAL)
                Y_goal = np.append(Y_goal, Y_GOAL)
                THETA_goal = np.append(THETA_goal, THETA_GOAL)


                status = robotFeedbackControl(velPub, x, y, theta, X_GOAL, Y_GOAL, radians(THETA_GOAL))
                if status == 'Goal Position reached':
                    robotStop(velPub)

                    '''
                    np.savetxt(LOG_DIR+'/X_traj.csv', X_traj, delimiter=' , ')
                    np.savetxt(LOG_DIR+'/Y_traj.csv', Y_traj, delimiter=' , ')
                    np.savetxt(LOG_DIR+'/THETA_traj.csv', THETA_traj, delimiter=' , ')
                    np.savetxt(LOG_DIR+'/X_goal.csv', X_goal, delimiter=' , ')
                    np.savetxt(LOG_DIR+'/Y_goal.csv', Y_goal, delimiter=' , ')
                    np.savetxt(LOG_DIR+'/THETA_goal.csv', THETA_goal, delimiter=' , ')
                    '''

                    rospy.signal_shutdown('End of testing')

    except rospy.ROSInterruptException:
        robotStop(velPub)
        print('Simulation terminated')
        pass
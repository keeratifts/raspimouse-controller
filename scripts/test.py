#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt, degrees
from nav_msgs.msg import Odometry
import numpy as np

class RobotFeedback:
    def __init__(self, robot):
        self.robot = robot
        self.odom = rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose = Odometry().pose.pose.position
        self.theta = 0

    def update_pose(self, msg):
        self.pose = msg.pose.pose.position
        self.pose.x = round(self.pose.x, 2)
        self.pose.y = round(self.pose.y, 2)
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.theta = degrees(round(self.theta, 4))

    def callback(self):
        return np.array([self.pose.x, self.pose.y, self.theta])

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous=False)
        raspi = RobotFeedback('raspi_0')
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            print (raspi.callback())
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


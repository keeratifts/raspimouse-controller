#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class raspimouse:
    def __init__(self):
        rospy.init_node('timer_node', anonymous=True)
        self.velocity_pub0 = rospy.Publisher("/raspi_0/cmd_vel", Twist, queue_size=10)
        self.velocity_pub1 = rospy.Publisher("/raspi_1/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber('raspi_0/odom',Odometry, self.callback)
        self.rate = rospy.Rate(10)

    def callback(self, msg):
        self.x = msg.pose.pose.position.x 
        self.y = msg.pose.pose.position.y

    def forward_mov(self, linear_x, angular_z, time):
        twist = Twist()

        twist.linear.x = linear_x
        twist.angular.z = angular_z

        for i in range(0, int(time)):
            self.velocity_pub0.publish(twist)
            self.velocity_pub1.publish(twist)
            self.rate.sleep()
            rospy.loginfo("x:%.2f, y:%.2f", self.x, self.y)

    def backward_mov(self, linear_x, angular_z, time):
        twist = Twist()

        twist.linear.x = -linear_x
        twist.angular.z = angular_z

        for i in range(0, int(time)):
            self.velocity_pub0.publish(twist)
            self.velocity_pub1.publish(twist)
            self.rate.sleep()
            rospy.loginfo("x:%.2f, y:%.2f", self.x, self.y)
        
            
if __name__ == '__main__':
    try:
        raspi = raspimouse()

        while not rospy.is_shutdown():

            movement = input('forward or backward: ')

            if movement == 'forward':
                sec = float(input("sec: "))
                raspi.forward_mov(0.3, 0.0, sec*10)
                raspi.forward_mov(0.0, 0.0, 1)
                rospy.sleep(0)

            if movement == 'backward':
                sec = float(input("sec: "))
                raspi.backward_mov(0.3, 0.0, sec*10)
                raspi.backward_mov(0.0, 0.0, 1)
                rospy.sleep(0)

            else:
                print ('re-input')

    except rospy.ROSInterruptException: pass

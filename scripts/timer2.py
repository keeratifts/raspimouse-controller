#! /usr/bin/env python3
  
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
  
class raspimouse:
    def __init__(self):
        rospy.init_node('raspi_duration_control', anonymous=True)
        self.twist_pub0 = rospy.Publisher('/raspi_0/cmd_vel', Twist, queue_size=100)
        rospy.Subscriber('/raspi_0/odom',Odometry, self.poseCallback)
        self.rate = rospy.Rate(10)
        #self.twist_pub1 = rospy.Publisher('/raspi_1/cmd_vel', Twist, queue_size=10)
        #rospy.Timer(rospy.Duration(1.0), self.timerCallback) #timercallback will be called every 1.0 second interval

        '''
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.twist_pub0.publish(twist)
        #self.twist_pub1.publish(twist)
        ''' 
    def poseCallback(self, msg):
        rospy.loginfo("x:%.2f, y:%.2f", msg.pose.pose.position.x, msg.pose.pose.position.y)

    def timerCallback(self):
        self.setMoveVector(0.5, 0.0, 20) #2seconds
        self.setMoveVector(0.1, 1.0, 17) #1.7seconds
        self.setMoveVector(0.5, 0.0, 20)
        self.setMoveVector(0.1, 1.0, 17)
        self.setMoveVector(0.5, 0.0, 20)
        self.setMoveVector(0.1, 1.0, 17)
        self.setMoveVector(0.5, 0.0, 20)
        self.setMoveVector(0.1, 1.0, 17)
        self.setMoveVector(0.0, 0.0, 1)
 
    def setMoveVector(self, linear_x, angular_z, cnt):
        twist = Twist()
        #r = rospy.Rate(10) #timers run 10 times per second
  
        twist.linear.x = linear_x
        twist.angular.z = angular_z
  
        for i in range(0, cnt):
            self.twist_pub0.publish(twist)
            #self.twist_pub1.publish(twist)
            self.rate.sleep() #100millisec
  
if __name__ == '__main__':
  
    try:
        raspi = raspimouse()
        raspi.timerCallback()
        #rospy.spin() infinite_loop non stop node
        rospy.sleep(0) 
    except rospy.ROSInterruptException: pass
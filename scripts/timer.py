#! /usr/bin/env python3
  
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
  
class raspimouse:
    def __init__(self):
        rospy.init_node('raspi_duration_control', anonymous=True)
        self.twist_pub0 = rospy.Publisher('/raspi_0/cmd_vel', Twist, queue_size=10)
        self.twist_pub1 = rospy.Publisher('/raspi_1/cmd_vel', Twist, queue_size=10)
        rospy.Timer(rospy.Duration(1.0), self.timerCallback)
  
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.twist_pub0.publish(twist)
        self.twist_pub1.publish(twist)
  
  
    def timerCallback(self, event):
        self.setMoveVector(0.3, 0.0, 20)
        self.setMoveVector(0.1, 1.0, 17)
        self.setMoveVector(0.3, 0.0, 20)
        self.setMoveVector(0.1, 1.0, 17)
        self.setMoveVector(0.3, 0.0, 20)
        self.setMoveVector(0.1, 1.0, 17)
        self.setMoveVector(0.3, 0.0, 20)
        self.setMoveVector(0.1, 1.0, 17)
 
    def setMoveVector(self, linear_x, angular_z, cnt):
        twist = Twist()
        r = rospy.Rate(10)
  
        twist.linear.x = linear_x
        twist.angular.z = angular_z
  
        for i in range(0, cnt):
            self.twist_pub0.publish(twist)
            self.twist_pub1.publish(twist)
            r.sleep()
  
if __name__ == '__main__':
  
    try:
        raspi = raspimouse()
        rospy.spin()
    except rospy.ROSInterruptException: pass
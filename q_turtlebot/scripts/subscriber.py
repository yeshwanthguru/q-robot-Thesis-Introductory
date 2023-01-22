#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def subscriberCallback(msg):
    rospy.loginfo("Received velocity command: [%f, %f, %f, %f, %f, %f]", 
                  msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
    # Use the received velocity command to control the turtle
    # ...

if __name__ == '__main__':
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber("/turtle1/cmd_vel", Twist, subscriberCallback)
    rospy.spin()


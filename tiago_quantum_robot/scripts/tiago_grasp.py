#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseStamped

def callback(data):
    # Invert the received pose
    inverted_pose = Pose()
    inverted_pose.position.x = -data.pose.position.x
    inverted_pose.position.y = -data.pose.position.y
    inverted_pose.position.z = -data.pose.position.z
    inverted_pose.orientation.x = -data.pose.orientation.x
    inverted_pose.orientation.y = -data.pose.orientation.y
    inverted_pose.orientation.z = -data.pose.orientation.z
    inverted_pose.orientation.w = data.pose.orientation.w
    
    # Publish the inverted pose on the /grasp_pose topic
    grasp_pose_pub.publish(PoseStamped(header=data.header, pose=inverted_pose))
    
    # Log the inverted pose to a file
    with open('inverted_poses.log', 'a') as f:
        f.write(str(inverted_pose) + '\n')

if __name__ == '__main__':
    rospy.init_node('pose_inverter', anonymous=True)
    detected_pose_sub = rospy.Subscriber('/detected_aruco_pose', PoseStamped, callback)
    grasp_pose_pub = rospy.Publisher('/grasp_pose', PoseStamped, queue_size=1)
    rospy.spin()


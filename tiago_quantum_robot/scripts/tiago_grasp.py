#!/usr/bin/env python

import rospy
import os
from geometry_msgs.msg import Pose, PoseStamped

LOG_FILE_PATH = '/home/yg/tiago_public_ws/src/tiago_quantum_robot/data/grasp_pose.log'

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
    
    # Log the inverted pose in the terminal
    rospy.loginfo("Inverted pose:\n%s", inverted_pose)
    
    # Log the inverted pose in the file
    with open(LOG_FILE_PATH, 'a') as f:
        f.write(str(inverted_pose) + '\n')

if __name__ == '__main__':
    # Create the directory for the log file if it doesn't exist
    log_dir = os.path.dirname(LOG_FILE_PATH)
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    rospy.init_node('pose_inverter', anonymous=True)
    detected_pose_sub = rospy.Subscriber('/detected_aruco_pose', PoseStamped, callback)
    grasp_pose_pub = rospy.Publisher('/grasp_pose', PoseStamped, queue_size=1)
    rospy.spin()



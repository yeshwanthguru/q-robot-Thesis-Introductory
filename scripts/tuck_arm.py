#!/usr/bin/env python

import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class RobotControl:
    def __init__(self):
        self.head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)

    def lower_head(self):
        rospy.loginfo("Moving head down")
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, -0.70]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        
        while not rospy.is_shutdown():
            self.head_cmd.publish(jt)
            rospy.sleep(0.1)  # Adjust the sleep duration as desired

if __name__ == "__main__":
    rospy.init_node("robot_control")
    robot = RobotControl()
    robot.lower_head()


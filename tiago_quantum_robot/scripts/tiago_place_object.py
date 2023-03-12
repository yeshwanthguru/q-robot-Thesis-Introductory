#!/usr/bin/env python

import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class RobotControl:
    def __init__(self):
        self.head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        self.torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)
        self.play_m_as = actionlib.SimpleActionClient("play_motion", PlayMotionAction)

    def tuck_arm(self):
        rospy.init_node("tuck_arm")
        rospy.loginfo("Waiting for play_motion...")
        self.play_m_as.wait_for_server()
        rospy.loginfo("...connected.")

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(3.0)

        rospy.loginfo("Tuck arm...")
        goal = PlayMotionGoal()
        goal.motion_name = 'home'
        goal.skip_planning = False

        self.play_m_as.send_goal(goal)
        self.play_m_as.wait_for_result(rospy.Duration(10.0))
        rospy.loginfo("Arm tucked.")

    def lift_torso(self):
        rospy.loginfo("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.34]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)
        rospy.loginfo("Done.")

    def lower_head(self):
        rospy.loginfo("Moving head down")
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, -0.75]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        rospy.loginfo("Done.")

    def prepare_robot(self):
        rospy.loginfo("Stopping arm_controller...")
        self.play_m_as.cancel_all_goals()
        rospy.sleep(1.0)  # wait for arm_controller to stop
        rospy.loginfo("Unfold arm safely")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'prepare_grasp'
        pmg.skip_planning = False
        self.play_m_as.send_goal_and_wait(pmg)
        rospy.loginfo("Done.")

        self.lift_torso()
        self.lower_head()

        rospy.loginfo("Robot prepared.")


if __name__ == "__main__":
    robot = RobotControl()
    robot.tuck_arm()
    robot.prepare_robot()


#!/usr/bin/env python
import rospy
import actionlib

from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

global decision_made
# define a global variable to store the pose of the object
decision_made = None

class Robot:
    def __init__(self):
        self.play_m_as = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.play_m_as.wait_for_server()

    def prepare_robot(self, decision_made):
        rospy.loginfo("Stopping arm_controller...")
        rospy.sleep(1.0) # wait for arm_controller to stop
        rospy.loginfo("Unfold arm safely")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'prepare_grasp'
        pmg.skip_planning = False
        self.play_m_as.send_goal(pmg)
        self.play_m_as.wait_for_result()
        rospy.loginfo("Grasp prepared.")

    def pick_pose(self):
        rospy.loginfo("Picking final pose")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pick_final_pose'
        pmg.skip_planning = False
        self.play_m_as.send_goal(pmg)
        self.play_m_as.wait_for_result()

def callback(data):
    global decision_made
    decision_made = data
    print(decision_made)
    robot.prepare_robot(decision_made)
    robot.pick_pose()
    pub = rospy.Publisher('/decision_achieved', String, queue_size=10)
    pub.publish("Decision achieved.")

def listener():
    rospy.init_node('listener', anonymous=True)
    global robot
    global grasp_pose_sub
    robot = Robot()
    grasp_pose_sub = rospy.Subscriber('/decision_made', String, callback)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException as e:
        rospy.logwarn("Exception occurred: {}".format(e))

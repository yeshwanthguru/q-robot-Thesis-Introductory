import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        self.play_m_as = actionlib.SimpleActionClient('play_motion', PlayMotionAction)
        rospy.loginfo("Waiting for play_motion action server...")
        self.play_m_as.wait_for_server()
        rospy.loginfo("play_motion action server found!")

    def prepare_robot(self, decision):
        rospy.loginfo("Stopping arm_controller...")
        rospy.sleep(1.0) # wait for arm_controller to stop
        rospy.loginfo("Unfold arm safely")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'home_right'
        pmg.skip_planning = False
        self.play_m_as.send_goal(pmg)
        self.play_m_as.wait_for_result()
        rospy.loginfo("Grasp prepared.")
        return decision

if __name__ == '__main__':
    controller = RobotController()
    decision = True  # Set your decision value here
    controller.prepare_robot(decision)

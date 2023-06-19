#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import control_msgs.msg
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Quaternion

# Global variables
robot = None
move_group = None
grasp_pose = None
object_pose = None

# Flag to indicate if pose has been received
pose_received = False
grasped = False
aruco_pose_in_base = None

# Define the desired values here
z_axis_offset = 0.3
y_axis_offset = 0.5

def close_gripper():
    pub_gripper_controller = rospy.Publisher('/gripper_right_controller/command', JointTrajectory, queue_size=1)
    for i in range(10):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['gripper_right_left_finger_joint', 'gripper_right_right_finger_joint']
        trajectory_points = JointTrajectoryPoint()
        trajectory_points.positions = [0.0, 0.0]
        trajectory_points.time_from_start = rospy.Duration(1.0)
        trajectory.points.append(trajectory_points)
        pub_gripper_controller.publish(trajectory)
        rospy.loginfo_once('close_gripper')
        rospy.sleep(0.1)

def open_gripper():
    pub_gripper_controller = rospy.Publisher('/gripper_right_controller/command', JointTrajectory, queue_size=1)
    for i in range(10):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['gripper_right_left_finger_joint', 'gripper_right_right_finger_joint']
        trajectory_points = JointTrajectoryPoint()
        trajectory_points.positions = [0.044, 0.044]
        trajectory_points.time_from_start = rospy.Duration(1.0)
        trajectory.points.append(trajectory_points)
        pub_gripper_controller.publish(trajectory)
        rospy.loginfo_once('open_gripper')
        rospy.sleep(0.1)

class RobotController:
    def __init__(self):
        self.play_m_as = actionlib.SimpleActionClient('play_motion', PlayMotionAction)
        rospy.loginfo("Waiting for play_motion action server...")
        self.play_m_as.wait_for_server()
        rospy.loginfo("play_motion action server found!")

    def prepare_robot(self, decision):
        rospy.loginfo("Stopping arm_controller...")
        rospy.sleep(1.0)  # Wait for arm_controller to stop
        rospy.loginfo("Unfold arm safely")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'home_right'
        pmg.skip_planning = False
        self.play_m_as.send_goal(pmg)
        self.play_m_as.wait_for_result()
        rospy.loginfo("Grasp prepared.")
        return decision


rospy.init_node('handover_object', anonymous=True)

# Close the gripper
open_gripper()

# Initialize the move group commander for the arm_right_torso
move_group = moveit_commander.MoveGroupCommander('arm_right_torso')

# Set the joint goal
joint_goal = [0.34, 1.34, -0.46, 1.43, 0.30, 0.91, 0.08, 0.00]

# Set the tolerance for the joint goal
tolerance = 0.01  # Adjust the tolerance as per your requirements

# Set the joint goal target
move_group.set_joint_value_target(joint_goal)

# Plan and execute the movement
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

# Check if the joint goal was reached within the specified tolerance
achieved_joint_state = move_group.get_current_joint_values()
is_goal_reached = all(abs(goal - achieved) < tolerance for goal, achieved in zip(joint_goal, achieved_joint_state))

if is_goal_reached:
    print("Joint goal reached successfully.")
else:
    print("Failed to reach the joint goal.")

# Create a TF2 buffer and listener
buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)
def callback(msg):
    global aruco_pose_in_base
    # Transform the received pose to the TF2 frame
    aruco_pose_in_base = tf2_geometry_msgs.do_transform_pose(msg, buffer.lookup_transform("base_footprint",
                                                                                           msg.header.frame_id,
                                                                                           rospy.Time(0),
                                                                                           rospy.Duration(1.0)))
# Callback function for grasp pose
def grasp_pose_callback(msg):
    global grasp_pose
    global object_pose
    global move_group
    global pose_received
    global grasped
    global aruco_pose_in_base

    if not pose_received:
        pose_received = True
        object_pose = msg
        object_pose.pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)
        object_pose.pose.position.z += z_axis_offset
        grasp_pose = msg.pose
        grasp_pose.position.y -= y_axis_offset
        object_pose.pose.position.y = -0.2
        rospy.loginfo('PickObject - attempting to reach pre-grasp pose')
        move_group.set_pose_target(object_pose.pose)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        rospy.loginfo('PickObject: reached pre-grasp pose')

        # Calculate the target pose for the end effector
        target_pose = aruco_pose_in_base.pose
        object_pose.pose.position.z = target_pose.position.z + (-0.04) # Set the z-axis value from target_pose
        move_group.set_pose_target(object_pose.pose)  # Use object_pose as the target
        plan1 = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        rospy.loginfo('PickObject: reached final grasp pose')
        rospy.sleep(1)
        rospy.loginfo('Grasping Object...')
        close_gripper()
        grasped = True

    elif grasped:
        # Lift object
        object_pose.pose.position.z += 0.1
        move_group.set_pose_target(object_pose.pose)
        plan2 = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        rospy.loginfo('lifting the object')
        grasped = False

        joint_goal = [0.34, 1.34, -0.46, 1.43, 0.30, 0.91, 0.08, 0.00]  # Joint values you provided
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        rospy.sleep(1)
        rospy.loginfo('Releasing Object...')
        open_gripper()
        rospy.sleep(1)
        rospy.loginfo('Returning Object to Initial Position...')
        move_group.set_pose_target(object_pose.pose)
        plan3 = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        

def aruco_pose_callback(msg):
    global aruco_pose_in_base
    aruco_pose_in_base = msg

rospy.Subscriber('/detected_aruco_pose', PoseStamped, aruco_pose_callback)
rospy.Subscriber('/grasp_pose', PoseStamped, grasp_pose_callback)

rospy.spin()
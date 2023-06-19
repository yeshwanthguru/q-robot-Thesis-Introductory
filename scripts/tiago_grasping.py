#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from tf2_geometry_msgs import *
from geometry_msgs.msg import Quaternion
import moveit_commander
import sys
from std_srvs.srv import Empty, EmptyResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from std_msgs.msg import Float32, Bool
import math
import control_msgs.msg

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

# Close the gripper
def close_gripper():
    pub_gripper_controller = rospy.Publisher('/gripper_right_controller/command', JointTrajectory, queue_size=1)
    for i in range(10):
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'gripper_right_left_finger_joint', 'gripper_right_right_finger_joint']
        trajectory_points = JointTrajectoryPoint()
        trajectory_points.positions = [0.0, 0.0]
        trajectory_points.time_from_start = rospy.Duration(1.0)
        trajectory.points.append(trajectory_points)
        pub_gripper_controller.publish(trajectory)
        rospy.loginfo('close_gripper')
        rospy.sleep(0.1)

# Open the gripper
def open_gripper():
    pub_gripper_controller = rospy.Publisher('/gripper_right_controller/command', JointTrajectory, queue_size=1)
    for i in range(10):
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'gripper_right_left_finger_joint', 'gripper_right_right_finger_joint']
        trajectory_points = JointTrajectoryPoint()
        trajectory_points.positions = [0.044, 0.044]
        trajectory_points.time_from_start = rospy.Duration(1.0)
        trajectory.points.append(trajectory_points)
        pub_gripper_controller.publish(trajectory)
        rospy.loginfo('open_gripper')
        rospy.sleep(0.1)

# Callback function for aruco pose
def callback(msg):
    global aruco_pose_in_base
    # Transform the received pose to the TF2 frame
    aruco_pose_in_base = tf2_geometry_msgs.do_transform_pose(msg, buffer.lookup_transform("base_footprint",
                                                                                           msg.header.frame_id,
                                                                                           rospy.Time(0),
                                                                                           rospy.Duration(1.0)))
# Reduce torso position by 0.1
def reduce_torso_position():
    pub_torso_controller = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)
    current_position = rospy.wait_for_message('/torso_controller/state', control_msgs.msg.JointTrajectoryControllerState).actual.positions[0]
    target_position = current_position - 0.3

    trajectory = JointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.joint_names = ['torso_lift_joint']
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.positions = [target_position]
    trajectory_point.time_from_start = rospy.Duration(1.0)
    trajectory.points.append(trajectory_point)

    pub_torso_controller.publish(trajectory)
    rospy.loginfo('Reducing Torso Position by 0.1')

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
        object_pose.pose.position.z = target_pose.position.z  # Set the z-axis value from target_pose
        move_group.set_pose_target(object_pose.pose)  # Use object_pose as the target
        plan1 = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        rospy.loginfo('PickObject: reached final grasp pose')
        reduce_torso_position()
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


def decision_callback(msg):
    global decision_received
    decision_received = msg.data

if __name__ == '__main__':
    rospy.init_node('tiago_handover_task')
    buffer = Buffer()
    listener = TransformListener(buffer)
    decision_received = False


    rospy.Subscriber('/activate_picking', Bool, decision_callback)

    while not decision_received:
        rospy.loginfo_once("Waiting for decision...")
        rospy.sleep(1.0)

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander('arm_right_torso')
    rospy.Subscriber('/grasp_pose', PoseStamped, grasp_pose_callback)
    rospy.Subscriber('/detected_aruco_pose', PoseStamped, callback)
    rospy.loginfo("PickObject - Services ready")

    rospy.spin()

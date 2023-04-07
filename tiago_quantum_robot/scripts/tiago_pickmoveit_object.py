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
import time

global robot
global move_group
global grasp_pose
global object_pose

# flag to indicate if pose has been received
pose_received = False
grasped = False

# define the desired values here
z_axis_offset = 0.1
y_axis_offset = 0.3

def reach_goal(data):
    global reach_goal
    positions = data.points[0].positions
    rospy.loginfo('Received Joint Positions: {}'.format(positions))
    reach_goal = positions

def close_gripper():
    pub_gripper_controller = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=1)
    for i in range(10):
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'gripper_left_finger_joint', 'gripper_right_finger_joint']
        trajectory_points = JointTrajectoryPoint()
        trajectory_points.positions = [0.0, 0.0]
        trajectory_points.time_from_start = rospy.Duration(1.0)
        trajectory.points.append(trajectory_points)
        pub_gripper_controller.publish(trajectory)
        rospy.loginfo('close_gripper')
        rospy.sleep(0.1)

    
def open_gripper():
    pub_gripper_controller = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=1)
    for i in range(10):
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'gripper_left_finger_joint', 'gripper_right_finger_joint']
        trajectory_points = JointTrajectoryPoint()
        trajectory_points.positions = [0.044, 0.044]
        trajectory_points.time_from_start = rospy.Duration(1.0)
        trajectory.points.append(trajectory_points)
        pub_gripper_controller.publish(trajectory)
        rospy.loginfo('open_gripper')
        rospy.sleep(0.1)

    
def grasp_pose_callback(msg):
    global grasp_pose    
    global object_pose
    global move_group
    global pose_received    
    global grasped
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
        rospy.loginfo('PickObject:reached')
        
        joint_goal = [0.31, 0.28, -0.42, -1.63, 1.71, -2.07, 1.25, 1.62]
        move_group.go(joint_goal, wait=True)
        move_group.stop()     
        rospy.sleep(1)
        rospy.loginfo('Grasping Object...')
        close_gripper()
        grasped = True
        
    elif grasped:
        # Lift object
        object_pose.pose.position.z += 0.1
        move_group.set_pose_target(object_pose.pose)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        rospy.loginfo('lifting the object')
        grasped = False
        
        joint_goal = reach_goal
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        rospy.sleep(1)        
        rospy.loginfo('Releasing Object...')
        open_gripper()
        rospy.sleep(1)
        rospy.loginfo('Returning Object to Initial Position...')
        move_group.set_pose_target(object_pose.pose)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        

if __name__ == '__main__':
    rospy.init_node('grasp_pose_subscriber')
    # initialize moveit commander
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    # define group of joint to adjust the height of TIAGo
    move_group = moveit_commander.MoveGroupCommander('arm_torso')      
    rospy.Subscriber('/grasp_pose', PoseStamped, grasp_pose_callback)
    rospy.Subscriber('/reach_goal', JointTrajectory, reach_goal)
    rospy.loginfo("PickObject - Services ready")
    rospy.spin()

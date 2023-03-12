#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Define a callback function to receive the grasp pose
def grasp_pose_callback(msg):
    global grasp_pose
    grasp_pose = msg.pose

if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('tiago_grasp_node')

        # Initialize the Tiago robot interface
        robot = RobotCommander()
        scene = PlanningSceneInterface()
        arm_group = MoveGroupCommander('arm')

        # Subscribe to the grasp pose topic
        grasp_pose = None
        rospy.Subscriber('/grasp_pose', PoseStamped, grasp_pose_callback)

        # Wait for the grasp pose to be received
        while not rospy.is_shutdown() and grasp_pose is None:
            rospy.sleep(0.1)

        # Convert the grasp pose from Quaternion to Euler angles
        q = [grasp_pose.orientation.x, grasp_pose.orientation.y,
             grasp_pose.orientation.z, grasp_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        # Move the arm to the grasp pose
        arm_group.set_pose_target(grasp_pose)
        arm_group.go()

        # Close the gripper
        gripper_group = MoveGroupCommander('gripper')
        gripper_group.set_joint_value_target([0.0])
        gripper_group.go()

        # Lift the object
        grasp_pose.position.z += 0.1
        grasp_pose.orientation = quaternion_from_euler(roll, pitch, yaw)
        arm_group.set_pose_target(grasp_pose)
        arm_group.go()

    except rospy.ROSInterruptException:
        pass


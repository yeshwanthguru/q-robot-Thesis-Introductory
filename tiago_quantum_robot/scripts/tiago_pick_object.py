#!/usr/bin/env python

import rospy
import math
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


def grasp_pose_callback(data):
    global target_position
    target_position = [data.pose.position.x, data.pose.position.y, data.pose.position.z]


def move_tiago_arm():
    # Initialize MoveIt commander and set the arm group
    robot = moveit_commander.RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander('arm')

    # Create a transform listener to check for obstacles
    listener = tf.TransformListener()

    # Subscribe to the grasp_pose topic
    rospy.Subscriber('/grasp_pose', geometry_msgs.msg.PoseStamped, grasp_pose_callback)

    # Loop until the arm reaches the target position
    while not rospy.is_shutdown():
        # Check if there are any obstacles between the current and target positions
        try:
            listener.waitForTransform('base_link', 'arm_7_link', rospy.Time(0), rospy.Duration(0.1))
            (trans, rot) = listener.lookupTransform('base_link', 'arm_7_link', rospy.Time(0))
            distance = math.sqrt(trans[0] ** 2 + trans[1] ** 2 + trans[2] ** 2)
            if distance < 0.1: # Change this to the desired obstacle avoidance threshold
                rospy.logwarn('Obstacle detected!')
                return False
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        # Move the arm towards the target position
        if 'target_position' in globals():
            arm_group.set_position_target(target_position)
            arm_group.go(wait=True)

            # Check if the arm has reached the target position
            current_pose = arm_group.get_current_pose().pose
            distance = math.sqrt((target_position[0] - current_pose.position.x) ** 2 +
                                 (target_position[1] - current_pose.position.y) ** 2 +
                                 (target_position[2] - current_pose.position.z) ** 2)
            if distance < 0.05: # Change this to the desired distance threshold
                rospy.loginfo('Target position reached!')
                return True


if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('tiago_arm_control', anonymous=True)

        # Move the Tiago arm
        success = move_tiago_arm()

        if success:
            rospy.loginfo('Tiago arm successfully moved to target position.')
        else:
            rospy.logerr('Failed to move Tiago arm to target position.')

    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3.8

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Empty

object_pose = None
pick_up_pose_done = False
place_pose_done = False
service_called = False

def pose_of_the_object(msg):
    global object_pose, pick_up_pose_done, service_called
    if not service_called:
        object_pose = msg.pose
        pos_x = object_pose.position.x
        pos_y = object_pose.position.y
        pos_z = object_pose.position.z
        ori_x = object_pose.orientation.x
        ori_y = object_pose.orientation.y
        ori_z = object_pose.orientation.z
        ori_w = object_pose.orientation.w

        # Print the extracted pose data in the terminal
        rospy.loginfo("Aruco detected object pose:")
        rospy.loginfo("  position:")
        rospy.loginfo("    x: {}".format(pos_x))
        rospy.loginfo("    y: {}".format(pos_y))
        rospy.loginfo("    z: {}".format(pos_z))
        rospy.loginfo("  orientation:")
        rospy.loginfo("    x: {}".format(ori_x))
        rospy.loginfo("    y: {}".format(ori_y))
        rospy.loginfo("    z: {}".format(ori_z))
        rospy.loginfo("    w: {}".format(ori_w))

        # Call pick_up_pose function
        pick_up_pose()

def pick_up_pose():
    global pick_up_pose_done, object_pose
    if object_pose is not None:
        # Generate pick goal measurement
        pick_goal = PoseStamped()
        pick_goal.pose = object_pose
        pick_goal.header.frame_id = "base_link"
        rospy.loginfo("Pick goal generated")
        pick_up_pose_done = True
    else:
        rospy.loginfo("No object detected, cannot pick up")

    # Call place_pose function
    place_pose()

def place_pose():
    global place_pose_done, pick_up_pose_done, service_called
    if pick_up_pose_done is True:
        # Generate place goal measurement
        place_goal = PoseStamped()
        place_goal.pose = object_pose
        place_goal.header.frame_id = "destination"
        rospy.loginfo("Place goal generated")
        place_pose_done = True
        service_called = True

        # Call pick_gui service
        rospy.wait_for_service('/pick_gui')
        try:
            pick_gui = rospy.ServiceProxy('/pick_gui', Empty)
            resp = pick_gui()
            rospy.loginfo("Service pick_gui called")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

        # Terminate the script
        rospy.signal_shutdown("End of script")
    else:
        rospy.loginfo("No object detected and pick goal received, cannot Proceed")
        rospy.signal_shutdown("End of script")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    rospy.Subscriber('/detected_aruco_pose', PoseStamped, pose_of_the_object)
    listener()

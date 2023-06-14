#!/usr/bin/env python
"""
Distance_graspobject
===========

This script calculates the distance for the robot and the  handover object based on its pose. 
When the script receives the grasp pose of the handover object, it represent the object is in the grasp pose. 
However, if the object is not oriented correctly(grasp pose will not be recived  the Aruco marker cannot be detected), and the robot determines that the object is not in the grasping pose.

"""
import rospy
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from std_msgs.msg import Float32

def distance_graspobject():
    """
    Calculates and publishes the distance between tiago camera frame and handover object grasp pose.
    """

    # Initialize the ROS node
    rospy.init_node('Distance_graspobject')

    # Create a TF2 buffer and listener
    buffer = Buffer()
    listener = TransformListener(buffer)

    def callback(data):
        """
        Callback function for handling received handover objectpose messages. 
        It calculates the distance between the robot and the received pose based on the object's pose using the ArUco tag. 
        If the object is in a specific position, the distance is calculated. 
        If the object is being oriented, the distance is not calculated, indicating that the object is not in the grasping position. Finally, the distance values are published on the appropriate topics.
        """

        try:
            # Transform the camera frame to the robot's base footprint frame
            gripper_pose = PoseStamped()
            gripper_pose.header.frame_id = "xtion_rgb_frame"
            gripper_pose.pose.position.x = 0
            gripper_pose.pose.position.y = 0
            gripper_pose.pose.position.z = 0
            gripper_pose.pose.orientation.x = 0
            gripper_pose.pose.orientation.y = 0
            gripper_pose.pose.orientation.z = 0
            gripper_pose.pose.orientation.w = 1

            # Perform the transformation from camera frame to base footprint frame
            gripper_pose_in_base = tf2_geometry_msgs.do_transform_pose(
                gripper_pose,
                buffer.lookup_transform("base_footprint", "xtion_rgb_frame", rospy.Time(0), rospy.Duration(1.0))
            )

            # Transform the received pose to the TF2 frame
            aruco_pose_in_base = tf2_geometry_msgs.do_transform_pose(
                data,
                buffer.lookup_transform("base_footprint", data.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            )

            # Calculate the Euclidean distance between the camera frame and the received pose
            distance = (
                (aruco_pose_in_base.pose.position.x - gripper_pose_in_base.pose.position.x) ** 2
                + (aruco_pose_in_base.pose.position.y - gripper_pose_in_base.pose.position.y) ** 2
                + (aruco_pose_in_base.pose.position.z - gripper_pose_in_base.pose.position.z) ** 2
            ) ** 0.5

            # Set the minimum and maximum distance values as per the requirement
            min_distance = 0.00
            max_distance = 0.66

            # Map the distance to a value between -1 and 1 with a center point of 0
            mapped_value = ((distance - min_distance) / (max_distance - min_distance)) * 2 - 1
            mapped_value = abs(mapped_value)
            mapped_value = min(1, mapped_value)

            # Publish the mapped value on the '/mapped_distance_graspobject' topic
            mapped_distance_pub.publish(mapped_value)

            # Publish the original distance on the '/distance_graspobject' topic
            distance_pub.publish(distance)

        except Exception as ex:
            rospy.logerr(ex)
            # Publish 0 if an exception occurs
            mapped_distance_pub.publish(0.0)
            distance_pub.publish(0.0)

    # Subscribe to the '/grasp_pose' topic and set the callback function
    rospy.Subscriber('/grasp_pose', PoseStamped, callback)

    # Publish the original distance on the '/distance_graspobject' topic
    distance_pub = rospy.Publisher('/distance_graspobject', Float32, queue_size=10)

    # Publish the mapped distance on the '/mapped_distance_graspobject' topic
    mapped_distance_pub = rospy.Publisher('/mapped_distance_graspobject', Float32, queue_size=10)

    # Continuously publish 0 on '/mapped_distance_graspobject' and '/distance_graspobject' topics if no messages are received
    rate = rospy.Rate(1)  # Adjust the rate as needed
    while not rospy.is_shutdown():
        mapped_distance_pub.publish(0)
        distance_pub.publish(Float32(0))
        rate.sleep()

if __name__ == '__main__':
    # Call the distance_graspobject function to start the distance calculation and publishing
    distance_graspobject()

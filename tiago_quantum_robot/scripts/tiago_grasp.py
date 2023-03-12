#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool


class GraspGenerator:
    def __init__(self):
        rospy.init_node('grasp_generator')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.detected_pose_sub = rospy.Subscriber('/detected_aruco_pose', PoseStamped, self.generate_grasp_pose)
        self.grasp_pose_pub = rospy.Publisher('/grasp_pose', PoseStamped, queue_size=1)
        self.object_height = None
        self.object_width = None
        self.object_depth = None
        self.object_size_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.get_object_size)

    def get_object_size(self, data):
        idx = None
        try:
            idx = data.name.index('aruco_cube')
        except ValueError:
            return
        if idx is not None:
            self.object_height = data.pose[idx].position.z
            self.object_width = data.pose[idx].position.x
            self.object_depth = data.pose[idx].position.y

    def generate_grasp_pose(self, detected_pose):
        if not all((self.object_height, self.object_width, self.object_depth)):
            return

        try:
            base_to_camera = self.tf_buffer.lookup_transform('base_link', detected_pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        object_pose = tf2_geometry_msgs.do_transform_pose(detected_pose, base_to_camera)

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.header.stamp = rospy.Time.now()

        # Define the grasp position as the center of the top face of the object
        grasp_pose.pose.position.x = object_pose.pose.position.x
        grasp_pose.pose.position.y = object_pose.pose.position.y
        grasp_pose.pose.position.z = object_pose.pose.position.z + (self.object_height / 2.0)

        # Define the grasp orientation as perpendicular to the top face of the object
        grasp_pose.pose.orientation.x = 0.0
        grasp_pose.pose.orientation.y = -1.0 / (2.0 ** 0.5)
        grasp_pose.pose.orientation.z = 0.0
        grasp_pose.pose.orientation.w = 1.0 / (2.0 ** 0.5)

        self.grasp_pose_pub.publish(grasp_pose)
        rospy.loginfo('Grasp pose: {}'.format(grasp_pose))


def main():
    gg = GraspGenerator()
    rospy.spin()


if __name__ == '__main__':
    main()


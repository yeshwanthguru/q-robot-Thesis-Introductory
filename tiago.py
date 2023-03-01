#!/usr/bin/env python3.8
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseArray , PoseStamped
from moveit_msgs.msg import PickupActionGoal, PlaceActionGoal


class PickGuiTrigger:
    def __init__(self):
        rospy.init_node('pick_gui_trigger')
        self.pick_gui_service = rospy.ServiceProxy('/pick_gui', Empty)
        rospy.loginfo("Initialized pick_gui_trigger")
        self.aruco_pose_sub = rospy.Subscriber("/detected_aruco_pose", PoseStamped, self.aruco_callback)
        self.grasp_poses_sub = rospy.Subscriber("/grasp_poses", PoseArray, self.callback_function)
        self.pickup_goal_sub = rospy.Subscriber("/pickup/goal", PickupActionGoal, self.pickup_callback)
        self.place_goal_sub = rospy.Subscriber("/place/goal", PlaceActionGoal, self.place_callback)
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.pick_gui_service()
                rospy.loginfo("Grasp poses detected, starting script...")
                rospy.loginfo("Made decision to pick the object")
                # Terminate the script after successful service call
                rospy.signal_shutdown("Service call successful.")
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: ", e)
            rospy.sleep(0.1)

    def aruco_callback(self, msg):
    # Extract the position and orientation data
        pose = msg.pose
        pos_x = pose.position.x
        pos_y = pose.position.y
        pos_z = pose.position.z
        ori_x = pose.orientation.x
        ori_y = pose.orientation.y
        ori_z = pose.orientation.z
        ori_w = pose.orientation.w

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
    def callback_function(self, msg):
        # Extract the first pose from the PoseArray
        pose = msg.poses[0]
        
        # Extract the position and orientation data
        pos_x = pose.position.x
        pos_y = pose.position.y
        pos_z = pose.position.z
        ori_x = pose.orientation.x
        ori_y = pose.orientation.y
        ori_z = pose.orientation.z
        ori_w = pose.orientation.w
        
        # Print the extracted pose data in the terminal
        rospy.loginfo("Grasp Object pose:")
        rospy.loginfo("  position:")
        rospy.loginfo("    x: {}".format(pos_x))
        rospy.loginfo("    y: {}".format(pos_y))
        rospy.loginfo("    z: {}".format(pos_z))
        rospy.loginfo("  orientation:")
        rospy.loginfo("    x: {}".format(ori_x))
        rospy.loginfo("    y: {}".format(ori_y))
        rospy.loginfo("    z: {}".format(ori_z))
        rospy.loginfo("    w: {}".format(ori_w))
    
    def pickup_callback(self, msg):
        print("Received pickup goal pose:pickuppose_recieved") # Print the pose to the terminal
        


    def place_callback(self, msg):
        print("Hi")

if __name__ == '__main__':
    PickGuiTrigger()
    rospy.spin()


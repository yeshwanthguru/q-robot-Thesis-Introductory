#!/usr/bin/env python3.8
import rospy
from std_srvs.srv import Empty
class PickGuiTrigger:
    def __init__(self):
        rospy.init_node('pick_gui_trigger')
        self.pick_gui_service = rospy.ServiceProxy('/pick_gui', Empty)
        rospy.loginfo("Initialized pick_gui_trigger")
        self.paused = False
        self.run()

    def run(self):
        try:
            rospy.wait_for_service('/pick_gui', timeout=5)
            rospy.loginfo("Grasp poses detected, starting script...")
            rospy.loginfo("Made decision to pick the object")
            self.pick_gui_service()
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: ", e)
        
        while not rospy.is_shutdown():
            key = input("Press 1 to trigger the picking system, 0 to pause, 2 to resume: ")
            if key == '1':
                try:
                    self.paused = False
                    self.pick_gui_service()
                    rospy.loginfo("Grasp poses detected, starting script...")
                    rospy.loginfo("Made decision to pick the object")
                except rospy.ServiceException as e:
                    rospy.loginfo("Service call failed: ", e)
            elif key == '0':
                self.paused = True
                rospy.loginfo("Script paused")
            elif key == '2':
                self.paused = False
                rospy.loginfo("Script resumed")
            else:
                rospy.loginfo("Invalid input")

            while self.paused and not rospy.is_shutdown():
                rospy.sleep(0.1)

if __name__ == '__main__':
    PickGuiTrigger()


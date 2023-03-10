#!/usr/bin/env python3.8
import rospy
import time
import qrobot
import rospy
import json
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from qrobot.qunits import SensorialUnit,QUnit
from qrobot.models import AngularModel
from qrobot.bursts import  OneBurst,ZeroBurst


rospy.init_node('Tiago_brain', anonymous=True)
Ts = 0.5  # sample every 500ms



object_pose = None
pick_up_pose_done = False
place_pose_done = False


#Sensorial unit ..

# Layer 0 - Unit 0
l0_unit0 = qrobot.qunits.SensorialUnit("l0_unit_0", Ts=5,)
# Layer 0 - Unit 1
l0_unit1 = qrobot.qunits.SensorialUnit("l0_unit_1", Ts=5,)
# Layer 0 - Unit 2
l0_unit2 = qrobot.qunits.SensorialUnit("l0_unit_2", Ts=5,)

def pose_of_the_object(msg):
    global object_pose, pick_up_pose_done
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
    l0_unit0.scalar_reading = 0.4
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
        l0_unit1.scalar_reading = 0.7
    else:
        rospy.loginfo("No object detected, cannot pick up")
    # Call place_pose function
    place_pose()

def place_pose():
    global place_pose_done, pick_up_pose_done
    if pick_up_pose_done is True:
        # Generate place goal measurement
        place_goal = PoseStamped()
        place_goal.pose = object_pose
        place_goal.header.frame_id = "destination"
        rospy.loginfo("Place goal generated")
        place_pose_done = True
        l0_unit2.scalar_reading = 0.7
        
    else:
        rospy.loginfo("No object detected and pick goal received, cannot Proceed")

        
        

#################################################################################Tiago_classical system########################################################################################

#Main function to trigger the robot classical picking system with the Multi qubit quantum state decision making
def tiago_classical_pickingsystem():
    topic_active = False
    # Create the qunit
    model = qrobot.models.AngularModel(n=3, tau=10)
    burst = qrobot.bursts.ZeroBurst()
    l1_unit0 = qrobot.qunits.QUnit("l1_unit0", model=model, burst=burst, Ts=10)
    # Set the input couplings for the qunit
    l1_unit0.set_input(0, "l0_unit0")
    l1_unit0.set_input(1, "l0_unit1")
    l1_unit0.set_input(2, "l0_unit2")
    l1_unit0.start()
    print(l1_unit0.in_qunits)
    l1_unit0.query=[0.6,0.5,0.8]
    statuses = []
    # Read statused and store it
    status = qrobot.qunits.redis_utils.redis_status()
    statuses.append(status)
    # Print output
    print(json.dumps(status, indent=1, sort_keys=True))
    
    while not rospy.is_shutdown():
        try:
            if not topic_active:
                # Subscribe to the rostopics for the sensory inputs
                rospy.Subscriber('/detected_aruco_pose', PoseStamped, pose_of_the_object)
                topic_active = True
                # Measure the combined state of all three sensorial unit
                shots = 1
                counts = model.measure(shots)
                result = counts
            #Quantum state decision to trigger the robot 
            if result >= Ts:
                
                try:
                    #Triggers the picking classical system
                    rospy.wait_for_service('/pick_gui', timeout=5)
                    pick_gui_service = rospy.ServiceProxy('/pick_gui', Empty)
                    rospy.loginfo("Grasp poses detected, starting script...")
                    rospy.loginfo("Made decision to pick the object")
                    pick_gui_service()
                except rospy.ServiceException as e:
                    rospy.loginfo("Service call failed: ", e)
            elif result < Ts:
                    #Decision not made so it runs with the default 
                    rospy.loginfo("Did not make decision ,started thinking")
                    #when no sensory input detected Goes with the default state input
                    rospy.loginfo("waiting for decision to pick the object") 
                    pass
        except:
            #no sensory connection it results with the warning
            rospy.logwarn("Timeout waiting for sensory inputs, exiting...")
            # Reinitialize the model to encode a new temporal window
            time.sleep(5)
            model.clear()
    
    l1_unit0.stop()
    qrobot.qunits.redis_utils.flush_redis()
    
if __name__ == '__main__':
    tiago_classical_pickingsystem()
    
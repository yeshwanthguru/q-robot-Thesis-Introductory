#!/usr/bin/env python3.8
import rospy
import json
import qrobot
from random import randint
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
from qrobot.models import AngularModel
from qrobot.bursts import ZeroBurst
from qrobot.qunits import QUnit, SensorialUnit

# Initialize ROS node
rospy.init_node('Tiago_brain', anonymous=True)


Sensorial_unit0 = SensorialUnit("Sensorial_unit0", Ts=0.1)
Sensorial_unit1 = SensorialUnit("Sensorial_unit1", Ts=0.1)
Sensorial_unit2 = SensorialUnit("Sensorial_unit2", Ts=0.1)

def objectpose_callback(msg):
    # Object pose measured by the robots camera detecting the aruco tag on the object(Generates the uncertain data based on the rostopic input).
    Sensorial_unit0.scalar_reading = randint(0, 1000) / 1000

def pick_callback(msg):
    # grasp_pose of the object which has been extracted from the object pose by inverting aruco tag transform to represent the grasp_pose of the object(Generates the uncertain data based on the rostopic input).\n    
    Sensorial_unit1.scalar_reading = randint(0, 1000) / 1000

def handover_callback(msg):
    # handover pose by the robot(Generates the uncertain data based on the rostopic input)
    Sensorial_unit2.scalar_reading = randint(0, 1000) / 1000
    
def tiago_brain():
    #topic_active = False
    
    # Subscribe  all topics(Object_pose,grasp_pose,reach_goal)
    detected_aruco_pose_subscriber = rospy.Subscriber("/detected_aruco_pose", PoseStamped, objectpose_callback)
    grasp_pose_subscriber = rospy.Subscriber('/grasp_pose', PoseStamped, pick_callback)
    reach_goal_subscriber = rospy.Subscriber('/reach_goal', JointTrajectory, handover_callback)
    
    # Wait until all topic subscribers are connected(sensory connection if ok proceed or else logs the warning)
    while not rospy.is_shutdown():
        if detected_aruco_pose_subscriber.get_num_connections() == 0:
            rospy.logwarn_once("'/detected_aruco_pose' sensor not connected")
            continue
        elif grasp_pose_subscriber.get_num_connections() == 0:
            rospy.logwarn_once("'/grasp_pose' sensor not connected")
            continue
        elif reach_goal_subscriber.get_num_connections() == 0:
            rospy.logwarn_once("'/reach_goal' sensor not connected")
            continue
        else:
            break  # All subscribers are active
    
    # Create a new QUnit that receives inputs from all three sensorial units
    qunit = QUnit(name="qunit", model=AngularModel(n=3, tau=25), burst=ZeroBurst(), Ts=0.5, in_qunits={
        0: Sensorial_unit0.id,
        1: Sensorial_unit1.id,
        2: Sensorial_unit2.id,
    })
    decision_made_published = False
    qunit.query = [0.8, 0.5, 0.6]
    Sensorial_unit0.start()
    Sensorial_unit1.start()
    Sensorial_unit2.start()
    qunit.start()

    # Create a publisher object for the '/decision_made' topic
    decision_made_publisher = rospy.Publisher('/decision_made', String, queue_size=10)

    while not rospy.is_shutdown():
        try:
            # Read status of the qunit output and store it 
            status = qrobot.qunits.redis_utils.redis_status()
            # Decision Block--->Extract output values for all QUnits
            # The following decision block extracts the output values for all QUnits. 
            # Decision block starts with a for loop that iterates over the items in the status dictionary.
            for key, value in status.items():
                # It checks if the key contains the string "class" and its corresponding value is "QUnit". 
                if ' class' in key and value == 'QUnit': 
                    # If this is true, it uses string slicing to retrieve the name of the QUnit output and stores it in qunit_name. 
                    qunit_name = key[:-6] 
                    # It then retrieves the output value from the status dictionary associated with this QUnit and stores it in qunit_output.
                    qunit_output = status.get(f'{qunit_name} output')
                    # If qunit_output is not None, it is converted to a float and compared to the  Ts.
                    if qunit_output is not None:
                        qunit_output = float(qunit_output)  
                        if qunit_output > qunit.Ts:
                            # Then, it prints the status dictionary and the value of qunit_name and qunit_output. 
                            print(json.dumps(status, indent=1, sort_keys=True))
                            print(f'{qunit_name} output:', qunit_output)                           
                            rospy.loginfo("Object is present let me pick the object")
                            qbrain_graph = qrobot.graph(status)           
                            qrobot.draw(qbrain_graph)
                            if not decision_made_published:
                                decision_made_published = True
                                # If the decision is made a string message "Object is Present" is passed through the topic /decision_made .
                                # based on subscribing the following topic ros side node executes the simulation working.
                                decision_made_publisher.publish(String(data="Object is Present"))
                                # Once the working is done it publishes the topic /decision_achieved 
                                # which i have subscribed in the qrobot node ,to restart the script(restart logic is just in case if place multiple objects on the table for handover task) 
                                rospy.Subscriber('/decision_achieved', String, restart_callback)
                                Sensorial_unit0.stop()
                                Sensorial_unit1.stop()
                                Sensorial_unit2.stop()
                                qunit.stop()
                                rospy.spin()
                            

            # Print output           
            print(json.dumps(status, indent=1, sort_keys=True))

            # Plot graph
            qbrain_graph = qrobot.graph(status)           
            qrobot.draw(qbrain_graph)

        except rospy.exceptions.ROSInterruptException:
            pass

    rospy.spin()

def restart_callback(msg):
    try:
        rospy.loginfo_once("Handover object executed")
        rospy.loginfo_once("waiting for the next command")
        tiago_brain()
    except rospy.exceptions.ROSInterruptException:
        pass


if __name__ == '__main__':
    tiago_brain()

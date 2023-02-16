#!/usr/bin/env python3.8
import rospy
import time
from std_srvs.srv import Empty
from qrobot.qunits import SensorialUnit,QUnit
from qrobot.models import AngularModel
from qrobot.bursts import  OneBurst
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import PickupActionGoal, PlaceActionGoal

# Initialize ROS node
rospy.init_node('Tiago_brain', anonymous=True)
n = 1  # number of qubits
tau = 1  # number of events for each temporal window
# model, burst and sampling time
model = AngularModel(n, tau)
burst = OneBurst()
Ts = 0.5  # sample every 500ms


#initiate the sensorial unit class
sensorial_unit1 = SensorialUnit("SensorialUnit1", Ts)
sensorial_unit2 = SensorialUnit("SensorialUnit2", Ts)
sensorial_unit3 = SensorialUnit("SensorialUnit3", Ts)

#initiate the Qunits class
qunit1 = QUnit("QUnit1", model, burst, Ts, in_qunits={0: sensorial_unit1.id})
qunit2 = QUnit("QUnit2", model, burst, Ts, in_qunits={0: sensorial_unit2.id})
qunit3 = QUnit("QUnit3", model, burst, Ts, in_qunits={0: sensorial_unit3.id})


#Grasp pose feedback from the ros after Aurco marker detector
def grasp_callback(msg):
    start_time = time.time() # record the start time
    # wait for the message to be received, with a timeout of 5 seconds
    while msg is None and time.time() - start_time < 5:
        pass

    if msg is not None:
       uncertain_value = 0.5
       sensorial_unit1.scalar_reading = uncertain_value
       model.encode(sensorial_unit1.scalar_reading, dim=0)
       rospy.loginfo("grasp pose of the object messgae recieved")
    else:
        # if the message is not received within the timeout, terminate the function
        rospy.logwarn("sensory input1 failed")
        return

def pickup_callback(msg):
    start_time = time.time() # record the start time
    # wait for the message to be received, with a timeout of 5 seconds
    while msg is None and time.time() - start_time < 5:
        pass

    if msg is not None:
       uncertain_value = 0.7
       sensorial_unit2.scalar_reading = uncertain_value
       model.encode(sensorial_unit2.scalar_reading, dim=0)
       rospy.loginfo("object pick command recieved to pick the object")
    else:
        # if the message is not received within the timeout, terminate the function
        rospy.logwarn("sensory input2 failed")
        return

def place_callback(msg):
    start_time = time.time() # record the start time
    # wait for the message to be received, with a timeout of 5 seconds
    while msg is None and time.time() - start_time < 5:
        pass

    if msg is not None:
       uncertain_value = 0.9
       sensorial_unit3.scalar_reading = uncertain_value
       model.encode(sensorial_unit3.scalar_reading, dim=0)
       rospy.loginfo("place call back message recieved to place the object")
    else:
        # if the message is not received within the timeout, terminate the function
        rospy.logwarn("sensory input3 failed")
        return

#Main function to trigger the robot classical picking system with the Multi qubit quantum state decision making
def tiago_classical_pickingsystem():
    topic_active = False
    while not rospy.is_shutdown():
        try:
            if not topic_active:
                # Subscribe to the rostopics for the sensory inputs
                rospy.Subscriber("/grasp_poses", PoseArray, grasp_callback)
                rospy.Subscriber('/pickup/goal', PickupActionGoal, pickup_callback)
                rospy.Subscriber('/place/goal', PlaceActionGoal, place_callback)
                topic_active = True
                # Measure the combined state of all three sensorial units
                shots = 1
                counts = model.measure(shots)
                result = counts
            #Quantum state decision to trigger the robot 
            if result['0'] >= 0.5:
                
                try:
                    #Triggers the picking classical system
                    rospy.wait_for_service('/pick_gui', timeout=5)
                    pick_gui_service = rospy.ServiceProxy('/pick_gui', Empty)
                    rospy.loginfo("Grasp poses detected, starting script...")
                    rospy.loginfo("Made decision to pick the object")
                    pick_gui_service()
                except rospy.ServiceException as e:
                    rospy.loginfo("Service call failed: ", e)
            elif result['0'] == 0:
                    #Decision not made so it runs with the default 
                    rospy.loginfo("Did not make decision ,started thinking")
                    #when no sensory input detected Goes with the default state input
                    rospy.loginfo("waiting for decision to pick the object")
                    sensorial_unit = SensorialUnit("SensorialUnit0", Ts)
                    rospy.loginfo("switch_signal =", sensorial_unit.scalar_reading)
                    # Measure the qubits
                    shots = 1
                    counts = model.measure(shots)
                    model.encode(sensorial_unit.scalar_reading , dim=0)
                    pass
        except:
            #no sensory connection it results with the warning
            rospy.logwarn("Timeout waiting for sensory inputs, exiting...")
            # Reinitialize the model to encode a new temporal window
            time.sleep(5)
            model.clear()

if __name__ == '__main__':
    tiago_classical_pickingsystem()

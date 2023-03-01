#!/usr/bin/env python3.8
import rospy
import time
import qrobot
import numpy as np
from std_srvs.srv import Empty
from qrobot.qunits import SensorialUnit,QUnit
from qrobot.models import AngularModel
from qrobot.bursts import  OneBurst,ZeroBurst
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import PickupActionGoal, PlaceActionGoal

# Initialize ROS node
rospy.init_node('Tiago_brain', anonymous=True)
n = 1  # number of qubits
tau = 1  # number of events for each temporal window
# model, burst and sampling time
model = qrobot.models.AngularModel(n, tau)
burst = OneBurst()
Ts = 0.5  # sample every 500ms


# Layer 0 - Unit 0
l0_unit0 = qrobot.qunits.SensorialUnit("l0_unit_0", Ts=0.1)
# Layer 0 - Unit 1
l0_unit1 = qrobot.qunits.SensorialUnit("l0_unit_1", Ts=0.1)
# Layer 0 - Unit 2
l0_unit2 = qrobot.qunits.SensorialUnit("l0_unit_2", Ts=0.1)


l0_unit0.scalar_reading = 0.5
l0_unit1.scalar_reading = 0.3
l0_unit2.scalar_reading = 0.4


#Grasp pose feedback from the ros after Aurco marker detector
def grasp_callback(msg):
    l0_unit0.scalar_reading = (np.random.randint(500, 1000) / 1000)
   

#Pick pose feedback after the grasp pose detected through the robot camera
def pickup_callback(msg):
    l0_unit1.scalar_reading = (np.random.randint(500, 1000) / 1000)
    

#Place pose feedback after the above two operation completed.
def place_callback(msg):
    l0_unit2.scalar_reading = (np.random.randint(500, 1000) / 1000)

print(l0_unit0.scalar_reading)
print(l0_unit1.scalar_reading)
print(l0_unit2.scalar_reading)

#Main function to trigger the robot classical picking system with the Multi qubit quantum state decision making
def tiago_classical_pickingsystem():
    topic_active = False
    # Layer 1 - Unit 0
    l1_unit0 = qrobot.qunits.QUnit(name="l1_unit0",model=AngularModel(n=1, tau=10),burst=OneBurst(),Ts=0.3,in_qunits={0: l0_unit0.id})
    # Layer 1 - Unit 1
    l1_unit1 = qrobot.qunits.QUnit(name="l1_unit1",model=AngularModel(n=1, tau=25),burst=ZeroBurst(),Ts=0.2,in_qunits={0: l0_unit0.id})    
    # Layer 1 - Unit 1
    l1_unit2 = qrobot.qunits.QUnit(name="l1_unit2",model=AngularModel(n=1, tau=25),burst=ZeroBurst(),Ts=0.2,in_qunits={0: l0_unit0.id})
    l1_unit0.query = 0.8
    l1_unit1.query = 0.8
    l1_unit2.query = 0.8
    l0_unit0.start()
    l0_unit1.start()
    l0_unit2.start()
    l1_unit0.start()
    l1_unit1.start()
    l1_unit2.start()
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
            if l1_unit0.query >= Ts:
                
                try:
                    #Triggers the picking classical system
                    rospy.wait_for_service('/pick_gui', timeout=5)
                    pick_gui_service = rospy.ServiceProxy('/pick_gui', Empty)
                    rospy.loginfo("Grasp poses detected, starting script...")
                    rospy.loginfo("Made decision to pick the object")
                    pick_gui_service()
                except rospy.ServiceException as e:
                    rospy.loginfo("Service call failed: ", e)
            elif l1_unit0.query < Ts:
                    #Decision not made so it runs with the default 
                    rospy.loginfo("Did not make decision ,started thinking")
                    #when no sensory input detected Goes with the default state input
                    rospy.loginfo("waiting for decision to pick the object")
                    l0_unit0.stop()
                    l0_unit1.stop()
                    l0_unit2.stop()
                    l1_unit0.stop()
                    l1_unit1.stop()
                    l1_unit2.stop()               
        except:
            #no sensory connection it results with the warning
            rospy.logwarn("Timeout waiting for sensory inputs, exiting...")
            # Reinitialize the model to encode a new temporal window
            time.sleep(5)
            model.clear()

if __name__ == '__main__':
    tiago_classical_pickingsystem()
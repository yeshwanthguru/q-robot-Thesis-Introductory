#!/usr/bin/env python3.8
import rospy
import time
import random
import numpy as np
from std_srvs.srv import Empty
from qrobot.qunits import SensorialUnit
from qrobot.models import AngularModel
from qrobot.bursts import  OneBurst
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import PickupActionGoal, PlaceActionGoal

# Initialize ROS node
rospy.init_node('Tiago_brain', anonymous=True)
uncertain_value = (np.random.randint(500, 1000) / 1000)
n = 1  # number of qubits
tau = 1  # number of events for each temporal window
# model, burst and sampling time
model = AngularModel(n, tau)
burst = OneBurst()
Ts = 0.5  # sample every 500ms

# Initialize the sensorial units
sensorial_unit1 = SensorialUnit("SensorialUnit1", Ts, default_input=uncertain_value)
sensorial_unit2= SensorialUnit("SensorialUnit2", Ts, default_input=uncertain_value)
sensorial_unit3 = SensorialUnit("SensorialUnit3", Ts, default_input=uncertain_value)

def callback_function(msg):
    uncertain_value = (np.random.randint(500, 1000) / 1000)
    sensorial_unit1.scalar_reading = uncertain_value
   

def pickup_callback(msg):
    uncertain_value = (np.random.randint(500, 1000) / 1000)
    sensorial_unit2._scalar_reading = uncertain_value
    

def place_callback(msg):
    uncertain_value = (np.random.randint(500, 1000) / 1000)
    sensorial_unit3.scalar_reading = uncertain_value
   

def tiago_brain():
    topic_active = False
    while not rospy.is_shutdown():   
        if topic_active:
            # Subscribe to the rostopics
            rospy.Subscriber("/grasp_poses", PoseArray, callback_function)
            rospy.Subscriber('/pickup/goal', PickupActionGoal, pickup_callback)
            rospy.Subscriber('/place/goal', PlaceActionGoal, place_callback)
            topic_active = True
            model.encode(sensorial_unit1.scalar_reading , dim=0)
            model.encode(sensorial_unit2.scalar_reading, dim=0)
            model.encode(sensorial_unit3.scalar_reading , dim=0)
            shots = 3
            model.print_circuit()
            counts = model.measure(shots)
            print("result =", counts)
            print("switch_signal =", sensorial_unit1.scalar_reading,sensorial_unit2.scalar_reading,sensorial_unit3.scalar_reading)
            
        else:
            topic_active = False
            print("waiting for decision to pick the object")
            sensorial_unit = SensorialUnit("SensorialUnit0", Ts, default_input=0)
            sensorial_unit.scalar_reading = 0
            print("switch_signal =", sensorial_unit.scalar_reading)
            model.encode(sensorial_unit.scalar_reading , dim=0)
            # Measure the qubits
            shots = 1
            model.print_circuit()
            counts = model.measure(shots)
            print("Timeout waiting for sensory inputs, exiting...")
            # Reinitialize the model to encode a new temporal window
            time.sleep(5)
            model.clear()
        if callback_function and pickup_callback and pickup_callback:
            # Call the /pick_gui service
               try:
                  rospy.wait_for_service('/pick_gui', timeout=5)
                  pick_gui_service = rospy.ServiceProxy('/pick_gui', Empty)
                  print("Grasp poses detected, starting script...")
                  print("Made decision to pick the object")
                  model.print_circuit()
                  pick_gui_service()
               except rospy.ServiceException as e:
                  print("Service call failed: ", e)
                  time.sleep(5)
                  model.clear()
               
if __name__ == '__main__':
    tiago_brain()



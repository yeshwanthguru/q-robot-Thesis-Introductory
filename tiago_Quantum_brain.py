#!/usr/bin/env python3.8
import rospy
import time
import random
import json
import logging
import numpy as np
from std_srvs.srv import Empty
from qrobot.qunits import SensorialUnit, QUnit
from qrobot.models import AngularModel
from qrobot.bursts import  OneBurst
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import PickupActionGoal, PlaceActionGoal

logging.basicConfig(level=logging.WARNING)

# Initialize ROS node
rospy.init_node('Tiago_brain', anonymous=True)
n = 1  # number of qubits
tau = 1  # number of events for each temporal window
# model, burst and sampling time
model = AngularModel(n, tau)
burst = OneBurst()
Ts = 0.5  # sample every 500ms

sensorial_unit1 = SensorialUnit("SensorialUnit1", Ts, default_input= (np.random.randint(500, 1000) / 1000))
sensorial_unit2= SensorialUnit("SensorialUnit2", Ts, default_input= (np.random.randint(500, 1000) / 1000))
sensorial_unit3 = SensorialUnit("SensorialUnit3", Ts, default_input= (np.random.randint(500, 1000) / 1000))

def callback_function(msg):
    logging.warning(msg.header)
    for i in range(len(msg.poses)):
        logging.warning(msg.poses[i])
def pickup_callback(msg):
    logging.warning(msg)
def place_callback(msg):
    logging.warning(msg)

def tiago_brain():
    last_n_qubits = [0] * n  # initialize last_n_qubits with n elements set to 0
    while not rospy.is_shutdown():
        try:
            rospy.Subscriber("/grasp_poses", PoseArray, callback_function)    
            rospy.Subscriber('/pickup/goal', PickupActionGoal, pickup_callback)   
            rospy.Subscriber('/place/goal', PlaceActionGoal, place_callback)      
            shots = 1
            counts = model.measure(shots)
            result = counts
            last_n_qubits.append(result)
            last_n_qubits.pop(0) # remove oldest element from the list
            model.encode(sensorial_unit1.scalar_reading , dim=0)
            model.encode(sensorial_unit2.scalar_reading, dim=0)
            model.encode(sensorial_unit3.scalar_reading , dim=0)
            print(model.circ)
            print("Aggregated binary outcomes of the circuit:")
            print(json.dumps(counts, sort_keys=True, indent=4))           
            if all(last_n_qubits) == 1:
            # Call the /pick_gui service
                try:
                    rospy.wait_for_service('/pick_gui', timeout=1)
                    pick_gui_service = rospy.ServiceProxy('/pick_gui', Empty)
                    print("Grasp poses detected, starting script...")
                    print("Made decision to pick the object")
                    pick_gui_service()
                except rospy.ServiceException as e:
                    print("Service call failed: ", e)
            elif all(last_n_qubits) == 0 :
                # Print a message waiting for decision
                print("waiting for decision to pick the object")
                pass
        except rospy.ROSException as e:
            sensorial_unit = SensorialUnit("SensorialUnit0", Ts, default_input=0)
            sensorial_unit.scalar_reading = 0
            print("switch_signal =", sensorial_unit.scalar_reading)
            # Measure the qubits
            shots = 1
            counts = model.measure(shots)
            result = list(counts.values())[0]
            last_n_qubits.append(result)
            last_n_qubits.pop(0) # remove oldest element from the list
            print("last_n_qubits =", last_n_qubits)
            model.encode(sensorial_unit.scalar_reading , dim=0)
            print(model.circ)
            print("Timeout waiting for sensory inputs, exiting...")
            # Reinitialize the model to encode a new temporal window
            time.sleep(5)
            model.clear()
        
if __name__ == '__main__':
    tiago_brain()
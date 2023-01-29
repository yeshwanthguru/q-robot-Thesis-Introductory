#!/usr/bin/env python3.8
import rospy
import json
import time
import numpy as np
from std_srvs.srv import Empty
from qrobot.qunits import SensorialUnit
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



def callback_function(msg):
    sensorial_unit1 = SensorialUnit("SensorialUnit1", Ts)
    uncertain_value = (np.random.randint(500, 1000) / 1000)
    sensorial_unit1.scalar_reading  = uncertain_value
    print("Sensor reading in callback_function: ", sensorial_unit1.scalar_reading)
    model.encode(sensorial_unit1.scalar_reading , dim=0)
    model.print_circuit()
    
def pickup_callback(msg):
    sensorial_unit2= SensorialUnit("SensorialUnit2", Ts)
    uncertain_value = (np.random.randint(500, 1000) / 1000)
    sensorial_unit2.scalar_reading = uncertain_value
    print("Sensor reading in pickup_callback: ", sensorial_unit2.scalar_reading)
    model.encode(sensorial_unit2.scalar_reading , dim=0)
    model.print_circuit()

def place_callback(msg):
    sensorial_unit3 = SensorialUnit("SensorialUnit3", Ts)
    uncertain_value = (np.random.randint(500, 1000) / 1000)
    sensorial_unit3.scalar_reading = uncertain_value
    print("Sensor reading in place_callback:  ", sensorial_unit3.scalar_reading)
    model.encode(sensorial_unit3.scalar_reading , dim=0)
    model.print_circuit()

def tiago_brain():
    topic_active = False
    while not rospy.is_shutdown():
        try:
            if not topic_active:
                # Subscribe to the rostopics
                rospy.Subscriber("/grasp_poses", PoseArray, callback_function)
                rospy.Subscriber('/pickup/goal', PickupActionGoal, pickup_callback)
                rospy.Subscriber('/place/goal', PlaceActionGoal, place_callback)
                topic_active = True
                # Measure the combined state of all three sensorial units
                shots = 1
                counts = model.measure(shots)
                result = counts
                print("Aggregated binary outcomes of the circuit:")
                print(json.dumps(result, sort_keys=True, indent=4))

            if result['0'] > 0.5:
                try:
                    rospy.wait_for_service('/pick_gui', timeout=5)
                    pick_gui_service = rospy.ServiceProxy('/pick_gui', Empty)
                    print("Grasp poses detected, starting script...")
                    print("Made decision to pick the object")
                    pick_gui_service()
                except rospy.ServiceException as e:
                    print("Service call failed: ", e)
        except:
            print("waiting for decision to pick the object")
            sensorial_unit = SensorialUnit("SensorialUnit0", Ts)
            print("switch_signal =", sensorial_unit.scalar_reading)
            # Measure the qubits
            shots = 1
            counts = model.measure(shots)
            print("Aggregated binary outcomes of the circuit:")
            print(json.dumps(counts, sort_keys=True, indent=4))
            model.encode(sensorial_unit.scalar_reading , dim=0)
            print(model.circ)
            print("Timeout waiting for sensory inputs, exiting...")
            # Reinitialize the model to encode a new temporal window
            time.sleep(5)
            model.clear()

if __name__ == '__main__':
    tiago_brain()

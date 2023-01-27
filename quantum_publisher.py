#!/usr/bin/env python3.8
import rospy
import json
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import time
from qrobot.qunits import SensorialUnit, QUnit
from qrobot.models import AngularModel
from qrobot.bursts import  OneBurst

# Initialize ROS node
rospy.init_node('publisher', anonymous=True)
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
# Define the model, burst and sampling time
n = 3  # number of qubits
tau = 3  # number of events for each temporal window
# Define the model, burst and sampling time
model = AngularModel(n, tau)

burst = OneBurst()
Ts = 0.5  # sample every 500ms

# Create  the SensorialUnit class
sensorial_unit = SensorialUnit("SensorialUnit1", Ts, default_input=0)

# Create  the QUnit class
qunit = QUnit("QUnit1", model, burst, Ts, in_qunits={0: sensorial_unit.id})

def move_turtle():
    last_n_qubits = [0] * n  # initialize last_n_qubits with n elements set to 0
     
    while not rospy.is_shutdown():
        try:
            # Wait for the turtle's Pose message
            pose = rospy.wait_for_message("/turtle1/pose", Pose, timeout=1)
            # Update the sensorial_unit's scalar reading with 1
            sensorial_unit = SensorialUnit("SensorialUnit1", Ts, default_input=1)
            print("switch_signal =", sensorial_unit.scalar_reading)
            # Measure the qubits
            shots = 1
            counts = model.measure(shots)
            result = counts
            last_n_qubits.append(result) # add new result to the list
            last_n_qubits.pop(0) # remove oldest element from the list
            print("last_n_qubits =", last_n_qubits)
            # Output the result with print or in a log file
            model.encode(1 , dim=0)
            print("Process is started and turtle is moving")
            print(model.circ)
            print("Aggregated binary outcomes of the circuit:")
            print(json.dumps(counts, sort_keys=True, indent=4))
            # move turtle
            if all(last_n_qubits) == 1:
                # activate navigation system by publishing velocity command
                msg = Twist()
                msg.linear.x = 1.0
                msg.angular.z = 0.5
                pub.publish(msg)
            elif all(last_n_qubits) == 0:
            # deactivate navigation system by not publishing velocity command
                pass
        except rospy.ROSException as e:
            # Update the sensorial_unit's scalar reading with 0
            sensorial_unit = SensorialUnit("SensorialUnit1", Ts, default_input=0)
            print("switch_signal =", sensorial_unit.scalar_reading)
            # Output the result with print or in a log file
            shots = 1
            counts = model.measure(shots)
            result = counts
            last_n_qubits.append(result) # add new result to the list
            last_n_qubits.pop(0) # remove oldest element from the list
            print("last_n_qubits =", last_n_qubits)
            model.encode(0 , dim=0)
            print("turtle is not detected")
            print(model.circ)
            print("Aggregated binary outcomes of the circuit:")
            print(json.dumps(counts, sort_keys=True, indent=4))
            # Reinitialize the model to encode a new temporal window
            time.sleep(5)
            model.clear()
    

if __name__ == '__main__':
    move_turtle()
import rospy
import json
import time
import random
import math
from std_msgs.msg import Int32
from qrobot.models import AngularModel
from rospy import Rate

# Add the following lines to initialize variables
variable1 = 0
variable2 = 0
threshold = 5

def callback1(data):
    global variable1
    variable1 = data.data

def callback2(data):
    global variable2
    variable2 = data.data

# Initialize a AngularModel with single qubit:
n = 1  # single qubit
tau = 1  # 10 events for each temporal window (5 seconds in total)
model = AngularModel(n, tau)

t_index = 1
switch_signal = 0

#initialize rosnode
rospy.init_node('qrobot', anonymous=True)
rate = Rate(10) # 10 Hz

# Subscribe to two topics named "topic1" and "topic2"
rospy.Subscriber("/turtle1/cmd_vel", Int32, callback1)
rospy.Subscriber("/turtle1/pose", Int32, callback2)
pub = rospy.Publisher('odom', Int32, queue_size=10)

while True:
    # A switch ("ON/OFF" signal) is read by the python script as input
    switch_signal = int(input("Enter 1 for On and 0 for Off: "))
    print("switch_signal =", switch_signal)
    # This input is then encoded in the AngularModel
    model.encode(switch_signal, dim=0)
    # check if the value of variable1 and variable2 meet some criteria
    if variable1 > threshold and variable2 < threshold:
        pub.publish(1)  # publish message to start simulation
    # If we are not at the end of the temporal window:
    if t_index < tau:
        t_index += 1
    # If we are at the end of the temporal window:
    else:
        # Print the final circuit
        print(model.circ)
        # Measure the module
        result = model.measure()
        # Output the result with print or in a log file
        print("result =", result)
        # Reinitialize the model to encode a new temporal window
        t_index = 1
        model.clear()
    rate.sleep()
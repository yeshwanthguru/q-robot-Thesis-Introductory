import rospy
import json
import random
import numpy as np
import qrobot
from qrobot import qunits
from qrobot.models import AngularModel
from qrobot.qunits import SensorialUnit, QUnit
from qrobot.bursts import OneBurst
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_srv2.srv import Random, RandomRequest, RandomResponse

target_x = 0
target_y = 0

# Create an instance of the AngularModel class with n=1
model = qrobot.models.AngularModel(n=3,tau=1)

# Create an instance of the SensorialUnit class
sensorial_unit = SensorialUnit("sensorial_unit", Ts=1)

# Declare the publisher as global
vel_pub = None
client = None

def subscriber_callback(pose_msg):
    rospy.loginfo("Robot position: [%f, %f]", pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y)

    # Encode the current position into the quantum state using the SensorialUnit
    model.encode(pose_msg.pose.pose.position.x, dim=0)
    model.encode(pose_msg.pose.pose.position.y, dim=1)

    #  QUnit class
    burst = OneBurst()
    qunit = QUnit("qunit", model, burst, Ts=1)

    # Measure the quantum state using the QUnit
    shots = 1000000
    counts = model.measure(shots)

    # Print the aggregated binary outcomes of the circuit
    print("Aggregated binary outcomes of the circuit:")
    print(json.dumps(counts, sort_keys=True, indent=4))

    # Use the pose_msg to get the current position of Odom
    vel = Twist()
    # create the instance to call service
    get_target = RandomRequest()
    # create the variables calculating the distance each of x and y
    dx = target_x - pose_msg.pose.pose.position.x
    dy = target_y - pose_msg.pose.pose.position.y
    # if the distance between target and current position is less than 0.1,change the target
    if (dx**2 + dy**2) <= 0.1**2:
        # set the range of random value
        get_target.min = -6
        get_target.max = 6
        # call service to get the new random target
        resp = client.call(get_target)
        target_x = resp.x
        target_y = resp.y
        # calculate again to update the distance each of x and y
        dx = target_x - pose_msg.pose.pose.position.x
        dy = target_y - pose_msg.pose.pose.position.y
    # calculate velocity
    k = 1
    vel.linear.x = k * dx
    vel.linear.y = k * dy
   
def main():
    # Initialization
    rospy.init_node("qrobot_node")
    
    # AngularModel class with n=1
    global model
    model = qrobot.models.AngularModel(n=1,tau=1)

    #SensorialUnit class
    global sensorial_unit
    sensorial_unit = SensorialUnit("sensorial_unit", Ts=1)

    # QUnit class
    global burst
    burst = OneBurst()
    global qunit
    qunit = QUnit("qunit", model, burst, Ts=1)

    #subscriber to listen to the turtle's pose topic
    rospy.Subscriber("odom", Odometry, subscriber_callback)

    #publisher to send velocity commands to the turtle
    global vel_pub
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    
    #service client to call the random target service
    global target_client
    target_client = rospy.ServiceProxy("random_target", Random)

    # Spin
    rospy.spin()

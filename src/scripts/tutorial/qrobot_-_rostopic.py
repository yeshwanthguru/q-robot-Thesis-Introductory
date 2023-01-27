import qrobot
import json
import numpy as np
from qrobot.models import AngularModel
from qrobot.qunits import SensorialUnit, QUnit
from qrobot.bursts import OneBurst
from std_msgs.msg import String
import rospy
from turtlesim.msg import Pose
x = 0
y = 0
width_of_workspace = 11
height_of_workspace = -11
# Create an instance of the AngularModel class with n=1
model = qrobot.models.AngularModel(n=4,tau=1)

# Create an instance of the SensorialUnit class
sensorial_unit = SensorialUnit("sensorial_unit", Ts=1)

# Create an instance of the QUnit class
burst = OneBurst()
qunit = QUnit("qunit", model, burst, Ts=1)

def callback(data):
    # Get the turtle's x and y position
    x_scaled = x / width_of_workspace
    y_scaled = y / height_of_workspace
    model.encode(x_scaled, dim=0)
    model.encode(y_scaled, dim=1)


    # Measure the quantum state using the QUnit
    shots = 1000000
    counts = model.measure(shots)

    # Print the aggregated binary outcomes of the circuit
    print("Aggregated binary outcomes of the circuit:")
    print(json.dumps(counts, sort_keys=True, indent=4))

    # Publish the output back to ROS
    pub.publish(counts)

def listener():
    rospy.init_node('listener', anonymous=True)
    # Subscribe to the turtle's pose topic
    rospy.Subscriber("turtle1/pose", Pose, callback)
    # Create a publisher to publish the output
    global pub
    pub = rospy.Service("turtle1/output", String, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()

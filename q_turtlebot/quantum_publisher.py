#!/usr/bin/env python3.8
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import time
from qrobot.models import AngularModel

rospy.init_node('publisher', anonymous=True)
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

# Initialize a AngularModel with single qubit
n = 3  # single qubit
tau = 3  # 10 events for each temporal window (5 seconds in total)
model = AngularModel(n, tau)
Ts = 0.5  # sample every 500ms

def move_turtle():
    # Start a loop
    t_index = 1
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_message("/turtle1/pose", Pose, timeout=1)
            switch_signal = 1
            print("switch_signal =", switch_signal)
            # This input is then encoded in the AngularModel
            model.encode(switch_signal, dim=0)
            # If we are not at the end of the temporal window:
            if t_index < tau:
                t_index += 1
            else:
                # Print the final circuit
                print(model.circ)
                # Measure the module
                result = model.measure()
                # Output the result with print or in a log file
                print("result =", result)
                print("Process is started and turtle is moving")
                # Reinitialize the model to encode a new temporal window
                t_index = 0
                model.clear()
                # move turtle
                msg = Twist()
                # Set the linear and angular velocity values
                msg.linear.x = 1.0
                msg.angular.z = 0.5
                # Publish the velocity command
                pub.publish(msg)
                time.sleep(1)
        except rospy.ROSException as e:
            switch_signal = 0
            print("switch_signal =", switch_signal)
            # This input is then encoded in the AngularModel
            model.encode(switch_signal, dim=0)
            # If we are not at the end of the temporal window:
            if t_index < tau:
                t_index += 1
            else:
                # Print the final circuit
                print(model.circ)
                # Measure the module
                result = model.measure()
                # Output the result with print or in a log file
                print("result =", result)
                print("turtle is not detected")
                # Reinitialize the model to encode a new temporal window
                t_index = 0
                model.clear()
                time.sleep(1)

if __name__ == '__main__':
    move_turtle()



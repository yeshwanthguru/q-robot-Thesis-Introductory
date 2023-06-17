#!/usr/bin/env python3.8

"""
Tiago Brain
===========

This script represents the brain of a Tiago robot. It subscribes to a distance between the robot and the pick object, processes the sensorial data and encodes in the hilbert state,
and makes  measurement result  based on the processed data. It then publishes the decision to activate or deactivate the picking routine.

The script uses the qrobot library for quantum-like perception modeling for robotics.

"""

from typing import Optional
import rospy
import json
import qrobot
from std_msgs.msg import Float32, Bool  
from qrobot.models import AngularModel
from qrobot.bursts import ZeroBurst, OneBurst
from qrobot.qunits import QUnit, SensorialUnit

# Initialize ROS node
rospy.init_node('Tiago_brain', anonymous=True)


mapped_distance_s_unit = SensorialUnit(
    "mapped_distance_s_unit",
    Ts=0.1  # Sampling period of 100ms
)

mapped_distance_q_unit = QUnit(
    name="mapped_distance_q_unit",
    model=AngularModel(n=1, tau=4),  # This unit integrates two events of a single scalar data source (output every 1.0 seconds)
    burst=ZeroBurst(),
    Ts=0.5,  # Sample input every 0.5 seconds
    in_qunits={0: mapped_distance_s_unit.id},  # Single sensorial unit as the only input on the 0-th dimension of the model
    query=[0.0]  # Query for the model
)


def pick0bject_mapped_distance_callback(msg):
    """
    Callback function for the mapped distance subscriber which represents the distance between the robot and the pick object.

    Args:
        msg: Float32 message containing the mapped distance data.
    """
    mapped_distance = msg.data  # Assign the mapped distance value from the message

    if 0.1 <= mapped_distance <= 0.3:
        mapped_distance = 0.3
    elif 0.4 <= mapped_distance <= 0.6:
        mapped_distance = 0.5
    elif 0.7 <= mapped_distance <= 1.0:
        mapped_distance = 0.7

    mapped_distance_s_unit.scalar_reading = mapped_distance




def activate_picking():
    """
    Main function to activate the picking routine based on the pick object distance data.
    """

    # Subscribe to the distance data from the ROS topic.
    mapped_distance_subscriber = rospy.Subscriber("/pickobject_mapped_distance", Float32, pick0bject_mapped_distance_callback)
    # Publish the decision with the following topic.
    activate_picking_publisher = rospy.Publisher('/activate_picking', Bool, queue_size=10)  # Updated publisher data type
    # Waiting for the distance calculation ...........
    while not rospy.is_shutdown():
        if mapped_distance_subscriber.get_num_connections() == 0:
            rospy.logwarn_once("/pickobject_mapped_distance' sensor not connected")
            rospy.sleep(0.1)  # Wait for 100ms before checking again
            continue
        else:
            break

    # Start the loop threads of the units
    mapped_distance_s_unit.start()
    mapped_distance_q_unit.start()

    while not rospy.is_shutdown():
        try:
            # Get the qunit burst output
            qunit_output = QUnit.get_burst_output(mapped_distance_q_unit)
            if qunit_output is not None and qunit_output <= 0.5:  # Read the output with the threshold mechanism and make the decision to make the robot believe the object is present.
                rospy.loginfo("Object is present, let me pick the object")
                activate_picking_publisher.publish(True)  # Publish True for "Object is present"
            else:
                activate_picking_publisher.publish(False)  # Publish False for "Object is not present"

        except rospy.exceptions.ROSInterruptException:
            pass

    rospy.spin()


if __name__ == '__main__':
    activate_picking()

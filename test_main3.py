#!/usr/bin/env python3.8

"""
Tiago Brain
===========

This script represents the brain of a Tiago robot. It subscribes to a distance between the robot and the pick object, processes the sensorial data,
and makes decisions based on the processed data. It then publishes the decision to activate or deactivate the picking routine.

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
mapped_distance_s_unit1 = SensorialUnit(
    "mapped_distance_s_unit1",
    Ts=0.1  # Sampling period of 100ms
)

mapped_distance_s_unit2 = SensorialUnit(
    "mapped_distance_s_unit2",
    Ts=0.1  # Sampling period of 100ms
)

mapped_distance_q_unit = QUnit(
    name="mapped_distance_q_unit",
    model=AngularModel(n=3, tau=10),  # This unit integrates 10 events of a three scalar data source
    burst=OneBurst(),
    Ts=0.5,  # Sample input every 0.5 seconds
    in_qunits={0: mapped_distance_s_unit.id,1: mapped_distance_s_unit1.id,
        2: mapped_distance_s_unit2.id,},  # Single sensorial unit as the only input on the 0-th dimension of the model
    query=[0.0,0.0,0.0]  # Query for the model
)


def pick0bject_mapped_distance_callback(msg):
    """
    Callback function for the mapped distance subscriber which represents the distance between the robot and the pick object.

    Args:
        msg: Float32 message containing the mapped distance data.
    """
    mapped_distance = msg.data

    if 0.1 <= mapped_distance <= 0.3:
        mapped_distance = 0.3
    elif 0.4 <= mapped_distance <= 0.6:
        mapped_distance = 0.5
    elif 0.7 <= mapped_distance <= 1.0:
        mapped_distance = 0.7

    mapped_distance_s_unit.scalar_reading = mapped_distance

def handover0bject_mapped_distance_callback(msg):
    """
    Callback function for the mapped distance subscriber which represents the distance between the robot and the grasp object.

    Args:
        msg: Float32 message containing the mapped distance data.
    """
    mapped_distance1 = msg.data

    if 0.1 <= mapped_distance1 <= 0.3:
        mapped_distance1 = 0.2
    elif 0.4 <= mapped_distance1 <= 0.6:
        mapped_distance1 = 0.4
    elif 0.7 <= mapped_distance1 <= 1.0:
        mapped_distance1 = 0.8

    mapped_distance_s_unit1.scalar_reading = mapped_distance1

def human_mapped_distance_callback(msg):
    """
    Callback function for the mapped distance subscriber which represents the distance between the robot and the human.

    Args:
        msg: Float32 message containing the mapped distance data.
    """
    mapped_distance2 = msg.data

    if 0.1 <= mapped_distance2 <= 0.3:
        mapped_distance2 = 0.1
    elif 0.4 <= mapped_distance2 <= 0.6:
        mapped_distance2 = 0.4
    elif 0.7 <= mapped_distance2 <= 1.0:
        mapped_distance2 = 0.9

    mapped_distance_s_unit2.scalar_reading = mapped_distance2



def activate_picking():
    """
    Main function to activate the picking routine based on the pick object distance data.
    """

    # Subscribe to the distance data from the ROS topic.
    mapped_distance_subscriber = rospy.Subscriber("/pickobject_mapped_distance", Float32, pick0bject_mapped_distance_callback)
    mapped_distance_subscriber1 = rospy.Subscriber("/mapped_distance_graspobject", Float32, handover0bject_mapped_distance_callback)
    mapped_distance_subscriber2 = rospy.Subscriber("/human_mapped_distance", Float32, human_mapped_distance_callback)
    
    # Publish the decision with the following topic.
    activate_picking_publisher = rospy.Publisher('/activate_picking', Bool, queue_size=10)  # Updated publisher data type
    # Waiting for the distance calculation ...........
    while not rospy.is_shutdown():
        if mapped_distance_subscriber.get_num_connections() == 0:
            rospy.logwarn_once("/pickobject_mapped_distance' sensor not connected")
            rospy.sleep(0.1)  # Wait for 100ms before checking again
            continue
        elif mapped_distance_subscriber1.get_num_connections() == 0:
            rospy.logwarn_once("/mapped_distance_graspobject' sensor not connected")
            continue
        elif mapped_distance_subscriber2.get_num_connections() == 0:
            rospy.logwarn_once("'/human_mapped_distance' sensor not connected")
            continue
        else:
            break

    # Start the loop threads of the units
    mapped_distance_s_unit.start()
    mapped_distance_s_unit1.start()
    mapped_distance_s_unit2.start()
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

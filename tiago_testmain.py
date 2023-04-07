#!/usr/bin/env python
import rospy
import json
import sys
from random import randint
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
import qrobot.qunits.redis_utils
from qrobot.models import AngularModel
from qrobot.bursts import ZeroBurst, OneBurst
from qrobot.qunits import QUnit, SensorialUnit

# Initialize ROS node
rospy.init_node('Tiago_brain', anonymous=True)

# Set appropriate Qunit query values
Ts = 0.5

Sensorial_unit0 = SensorialUnit("Sensorial_unit0", Ts=0.1)
Sensorial_unit1 = SensorialUnit("Sensorial_unit1", Ts=0.1)
Sensorial_unit2 = SensorialUnit("Sensorial_unit2", Ts=0.1)


def objectpose_callback(msg):
    # Use actual sensor data from the robot
    Sensorial_unit0.scalar_reading = randint(0, 1000) / 1000

def pick_callback(msg):
    # Use actual sensor data from the robot
    Sensorial_unit1.scalar_reading = randint(0, 1000) / 1000

def handover_callback(msg):
    # Use actual sensor data from the robot
    Sensorial_unit2.scalar_reading = randint(0, 1000) / 1000

def tiago_brain():
    # Create a new QUnit that receives inputs from all three sensorial units
    Qunit = QUnit(name="Qunit", model=AngularModel(n=3, tau=25), burst=ZeroBurst(), Ts=0.2, in_qunits={
        0: Sensorial_unit0.id,
        1: Sensorial_unit1.id,
        2: Sensorial_unit2.id,
    })
    topic_active = False
    decision_made_published = False
    Qunit.query = [0.8, 0.5, 0.6]
    Sensorial_unit0.start()
    Sensorial_unit1.start()
    Sensorial_unit2.start()
    Qunit.start()
    statuses = []

    # Create a publisher object for the '/decision_made' topic
    decision_made_publisher = rospy.Publisher('/decision_made', String, queue_size=10)

    while not rospy.is_shutdown():
        try:
            if not topic_active:
                rospy.Subscriber("/detected_aruco_pose", PoseStamped, objectpose_callback)
                rospy.Subscriber('/grasp_pose', PoseStamped, pick_callback)
                rospy.Subscriber('/reach_goal', JointTrajectory, handover_callback)
                topic_active = True

            # Read status and store it
            status = qrobot.qunits.redis_utils.redis_status()
            statuses.append(status)

            # Extract output values for all QUnits
            for key, value in status.items():
                if ' class' in key and value == 'QUnit':
                    qunit_name = key[:-6]  # Remove ' output' suffix
                    qunit_output = status.get(f'{qunit_name} output')
                    if qunit_output is not None:
                        qunit_output = float(qunit_output)  # Convert to float
                        if qunit_output > Ts:
                            Sensorial_unit0.stop()
                            Sensorial_unit1.stop()
                            Sensorial_unit2.stop()
                            Qunit.stop()
                            print(json.dumps(status, indent=1, sort_keys=True))
                            print(f'{qunit_name} output:', qunit_output)
                            print("Object is present let me pick the object")
                            if not decision_made_published:
                                decision_made_published = True
                                decision_made_publisher.publish(String(data="Object is Present"))
                                rospy.Subscriber('/decision_achieved', String, restart_callback)
                                rospy.spin()

            # Print output
            print(json.dumps(status, indent=1, sort_keys=True))

            # Plot graph
            qbrain_graph = qrobot.graph(status)           
            qrobot.draw(qbrain_graph)

        except rospy.exceptions.ROSInterruptException:
            pass

def restart_callback(msg):
	try:
		tiago_brain()
	except rospy.exceptions.ROSInterruptException:
		pass

if __name__ == '__main__':
    tiago_brain()
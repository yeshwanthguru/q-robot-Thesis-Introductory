#!/usr/bin/env python3.8

import rospy
import json
import qrobot
from std_msgs.msg import Float32, String
from qrobot.models import AngularModel
from qrobot.bursts import ZeroBurst
from qrobot.qunits import QUnit, SensorialUnit

# Initialize ROS node
rospy.init_node('Tiago_brain', anonymous=True)

Sensorial_unit0 = SensorialUnit("Sensorial_unit0", Ts=0.1)

def objectpose_callback(msg):
    # Extract relevant data from message
    object_pose = msg.data   
    Sensorial_unit0.scalar_reading = object_pose

def tiago_brain():
    detected_aruco_pose_subscriber = rospy.Subscriber("/mapped_distance", Float32, objectpose_callback)
    
    while not rospy.is_shutdown():
        if detected_aruco_pose_subscriber.get_num_connections() == 0:
            rospy.logwarn_once("/mapped_distance' sensor not connected")
            continue
        else:
            break  

    qunit = QUnit(name="qunit", model=AngularModel(n=1, tau=2), burst=ZeroBurst(), Ts=0.5, in_qunits={
        0: Sensorial_unit0.id,
    })
    
    qunit.query = [0.3]
    Sensorial_unit0.start()
    qunit.start()

    decision_made_publisher = rospy.Publisher('/decision_made', String, queue_size=10)

    while not rospy.is_shutdown():
        try:
            status = qrobot.qunits.redis_utils.redis_status()
            for key, value in status.items():
                if ' class' in key and value == 'QUnit': 
                    qunit_name = key[:-6] 
                    
                    qunit_output = status.get(f'{qunit_name} output')
                    print(f'Qunit output after getting it is: {qunit_output}\n\n')

                    if qunit_output is not None:
                        qunit_output = float(qunit_output)  
                        if Sensorial_unit0.scalar_reading == 0:
                            print("The decision now is False")
                        elif qunit_output > qunit.Ts:
                            print(json.dumps(status, indent=1, sort_keys=True))
                            print(f'{qunit_name} output:', qunit_output)                           
                            rospy.loginfo("Object is present, let me pick the object")
                            decision_made_publisher.publish("True")  # Publish "True" for "Object is Present"
                      

            print(json.dumps(status, indent=1, sort_keys=True))

        except rospy.exceptions.ROSInterruptException:
            pass

    rospy.spin()

if __name__ == '__main__':
    tiago_brain()

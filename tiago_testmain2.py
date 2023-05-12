#!/usr/bin/env python3.8
import rospy
import json
import qrobot
from std_msgs.msg import Float32
from std_msgs.msg import String
from qrobot.models import AngularModel
from qrobot.bursts import ZeroBurst
from qrobot.qunits import QUnit, SensorialUnit

# Initialize ROS node
rospy.init_node('Tiago_brain', anonymous=True)


Sensorial_unit0 = SensorialUnit("Sensorial_unit0", Ts=0.1)

def objectpose_callback(msg):
    
    # extract relevant data from message
    object_pose = msg.data
    
    # store data in Sensorial_unit0
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
    
    decision_made_published = False
    qunit.query = [0.8]
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
                    if qunit_output is not None:
                        qunit_output = float(qunit_output)  
                        if qunit_output > qunit.Ts:
                            print(json.dumps(status, indent=1, sort_keys=True))
                            print(f'{qunit_name} output:', qunit_output)                           
                            rospy.loginfo("Object is present let me pick the object")
                            
                            if not decision_made_published:
                                decision_made_published = True
                                decision_made_publisher.publish(String(data="Object is Present"))
                                rospy.Subscriber('/decision_achieved', String, restart_callback)
                                rospy.spin()

            print(json.dumps(status, indent=1, sort_keys=True))
            

        except rospy.exceptions.ROSInterruptException:
            pass

    rospy.spin()

def restart_callback(msg):

    try:
        rospy.loginfo_once("Handover object on process")
        rospy.loginfo_once("waiting for the next command")
        tiago_brain()
    except rospy.exceptions.ROSInterruptException:
        pass

if __name__ == '__main__':
    tiago_brain()

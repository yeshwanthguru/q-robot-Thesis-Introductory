#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def recognize(image):
    # Convert the ROS Image message to OpenCV format
    cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
    
    # Display the image in a pop-up window
    cv2.imshow("Camera Feed", cv_image)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('camera_subscriber')
    
    # Create a CvBridge object
    bridge = CvBridge()
    
    # Create a subscriber for the camera topic
    sub_camera = rospy.Subscriber('/xtion/rgb/image_raw', Image, recognize)
    
    # Start the ROS event loop
    rospy.spin()


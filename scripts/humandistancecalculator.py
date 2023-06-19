#!/usr/bin/env python3.8
"""
Human_distance_publisher
===========

This script represents distance between the robot and the human based on the defined poses in the script.
As per the script based on the human hand move over than defined angle 45 degree represent the handover motion and  the distance has been calculated between the human body and the robot.

"""
import cv2
import mediapipe as mp
from mediapipe.python.solutions import pose as mp_pose
import math
import rospy
from std_msgs.msg import Float32
import threading

def calculate_distance(focal_length, known_width, pixel_width):
    """Calculate the distance based on focal length, known width, and pixel width.

    Args:
        focal_length (float): The focal length of the camera in pixels.
        known_width (float): The known width in centimeters.
        pixel_width (float): The width in pixels.

    Returns:
        float: The calculated distance.
    """
    return (known_width * focal_length) / pixel_width

def process_image(frame, pose, mp_drawing, focal_length, known_width, distance_publisher, mapped_distance_publisher):
    """Process the image and detect human poses.

    Args:
        frame (numpy.ndarray): The input image frame.
        pose (mediapipe.python.solutions.pose.Pose): The Pose model instance.
        mp_drawing: The drawing utilities from the MediaPipe library.
        focal_length (float): The focal length of the camera in pixels.
        known_width (float): The known width in centimeters.
        distance_publisher (rospy.Publisher): The publisher for distance values.
        mapped_distance_publisher (rospy.Publisher): The publisher for mapped distance values.
    """
    # Convert the image to RGB
    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process the image and detect human poses
    results = pose.process(image_rgb)

    # Draw the pose landmarks on the image
    image_out = frame.copy()
    if results.pose_landmarks:
        mp_drawing.draw_landmarks(image_out, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        # Calculate the hand angles
        right_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
        right_elbow = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW]
        right_wrist = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST]

        left_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
        left_elbow = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW]
        left_wrist = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST]

        right_shoulder_angle = math.degrees(math.atan2(right_elbow.y - right_shoulder.y, right_elbow.x - right_shoulder.x))
        right_elbow_angle = math.degrees(math.atan2(right_wrist.y - right_elbow.y, right_wrist.x - right_elbow.x))
        right_hand_angle = right_shoulder_angle - right_elbow_angle

        left_shoulder_angle = math.degrees(math.atan2(left_elbow.y - left_shoulder.y, left_elbow.x - left_shoulder.x))
        left_elbow_angle = math.degrees(math.atan2(left_wrist.y - left_elbow.y, left_wrist.x - left_elbow.x))
        left_hand_angle = left_shoulder_angle - left_elbow_angle

        # Check if the hand angles exceed the motion threshold
        motion_threshold = 45

        if right_hand_angle > motion_threshold or left_hand_angle > motion_threshold:
            calculate_and_publish_distance(results.pose_landmarks, image_out, focal_length, known_width, distance_publisher, mapped_distance_publisher)
        else:
            # No hand motion detected, publish 0
            distance_publisher.publish(0)
            mapped_distance_publisher.publish(0)

    # Show the output image
    cv2.imshow('Human Pose Estimation', image_out)

def calculate_and_publish_distance(landmarks, image_out, focal_length, known_width, distance_publisher, mapped_distance_publisher):
    """Calculate the distance, map it to a value between -1 and 1, and publish the results.

    Args:
        landmarks: The pose landmarks.
        image_out (numpy.ndarray): The output image frame.
        focal_length (float): The focal length of the camera in pixels.
        known_width (float): The known width in centimeters.
        distance_publisher (rospy.Publisher): The publisher for distance values.
        mapped_distance_publisher (rospy.Publisher): The publisher for mapped distance values.
    """
    # Calculate the distance
    x1 = landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].x * image_out.shape[1]
    x2 = landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].x * image_out.shape[1]
    pixel_width = abs(x2 - x1)
    distance = calculate_distance(focal_length, known_width, pixel_width)

    # Map the distance to a value between -1 and 1 with a center point of 0
    min_distance = 250  # Minimum distance in cm
    max_distance = 350  # Maximum distance in cm
    mapped_value = ((distance - min_distance) / (max_distance - min_distance)) * 2 - 1
    mapped_value = abs(mapped_value)
    mapped_value = min(1, mapped_value)

    # Publish the distance and mapped value
    distance_publisher.publish(distance)
    mapped_distance_publisher.publish(mapped_value)

    # Display the distance and mapped value on the image
    cv2.putText(image_out, f"Distance: {distance:.2f} cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(image_out, f"Mapped Value: {mapped_value:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

def main():
    """Main function to initialize ROS node, set up publishers, and process video frames."""
    rospy.init_node('human_distance_publisher', anonymous=True)
    distance_publisher = rospy.Publisher('human_distance', Float32, queue_size=10)
    mapped_distance_publisher = rospy.Publisher('/human_mapped_distance', Float32, queue_size=10)

    cap = cv2.VideoCapture(0)  # Use default camera
    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)
    mp_drawing = mp.solutions.drawing_utils  # Module for drawing landmarks

    # Known values for distance calculation
    known_width = 50  # Estimated average shoulder width in centimeters
    focal_length = 800  # Focal length of the camera in pixels

    def process_frames():
        """Thread target function to process video frames."""
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Process the current frame
            process_image(frame, pose, mp_drawing, focal_length, known_width, distance_publisher, mapped_distance_publisher)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Start a separate thread to process frames
    frame_thread = threading.Thread(target=process_frames)
    frame_thread.start()

    # Wait for the processing thread to finish
    frame_thread.join()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

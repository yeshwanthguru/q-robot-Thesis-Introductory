import cv2
import mediapipe as mp
import math
import rospy
from std_msgs.msg import Float32

def calculate_distance(focal_length_pixels, known_width_cm, avg_pixel_width):
    return (known_width_cm * focal_length_pixels) / avg_pixel_width

def main():
    cap = cv2.VideoCapture(0)  # Use default camera

    mp_drawing = mp.solutions.drawing_utils
    mp_objectron = mp.solutions.objectron

    objectron = mp_objectron.Objectron(
        static_image_mode=False, max_num_objects=1, min_detection_confidence=0.5,
        min_tracking_confidence=0.8, model_name='Cup'
    )

    focal_length_pixels = 800  # Focal length of the camera in pixels
    known_width_cm = 20  # Width of the cup in centimeters

    rospy.init_node('cup_detection_node')
    distance_pub = rospy.Publisher('cup_distance', Float32, queue_size=1)

    while not rospy.is_shutdown():
        try:
            ret, frame = cap.read()
            if not ret:
                break

            image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            results = objectron.process(image_rgb)

            if results.detected_objects:
                for detected_object in results.detected_objects:
                    mp_drawing.draw_landmarks(
                        frame, detected_object.landmarks_2d, mp_objectron.BOX_CONNECTIONS
                    )

                    # Convert landmarks to a list of dictionaries
                    landmarks = [
                        {
                            'x': landmark.x,
                            'y': landmark.y,
                            'z': landmark.z,
                            'visibility': landmark.visibility,
                        }
                        for landmark in detected_object.landmarks_2d.landmark
                    ]

                    # Calculate the distance to the object
                    if len(landmarks) > 1:
                        avg_pixel_width = abs(landmarks[0]['x'] - landmarks[1]['x'])
                        distance = calculate_distance(focal_length_pixels, known_width_cm, avg_pixel_width)
                        distance_pub.publish(distance)

            cv2.imshow('Cup Detection', frame)

            if cv2.waitKey(1) == ord('q'):
                break

        except Exception as e:
            print(f"Error: {str(e)}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

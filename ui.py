import cv2
import mediapipe as mp
import math

def calculate_distance(focal_length, known_width, pixel_width):
    return (known_width * focal_length) / pixel_width

def main():
    cap = cv2.VideoCapture(0)  # Use default camera (index 0)
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5, min_tracking_confidence=0.5)
    mp_drawing = mp.solutions.drawing_utils  # Module for drawing landmarks

    # Known values for distance calculation
    known_width = 20  # Width of the hand in centimeters
    focal_length = 800  # Focal length of the camera in pixels (example value)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Convert the image to RGB
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the image and detect hands
        results = hands.process(image_rgb)

        # Draw the hand landmarks on the image
        image_out = frame.copy()
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(image_out, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Calculate the distance
                x1 = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x * image_out.shape[1]
                x2 = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x * image_out.shape[1]
                pixel_width = abs(x2 - x1)
                distance = calculate_distance(focal_length, known_width, pixel_width)

                # Display the distance on the image
                cv2.putText(image_out, f"Distance: {distance:.2f} cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Show the output image
        cv2.imshow('Hand Detection', image_out)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

import cv2
import time
from qrobot.models import AngularModel

# Initialize a AngularModel with single qubit:
n = 1  # single qubit
tau = 1  # 10 events for each temporal window (5 seconds in total)
model = AngularModel(n, tau)

t_index = 1
switch_signal = 0

# Load the face detection model
face_cascade = cv2.CascadeClassifier("/home/yg/q-robot-Thesis-Introductory/.venv/lib/python3.8/site-packages/cv2/data/haarcascade_frontalface_default.xml")

# Start the camera
cap = cv2.VideoCapture(0)

while True:
    # Get the current frame from the camera
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the frame
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # Loop over the detected faces
    for (x, y, w, h) in faces:
        switch_signal = 1

    # If no faces are detected, set the switch signal to 0
    if len(faces) == 0:
        switch_signal = 0
        print("No human is around")
        time.sleep(5)

    # This input is then encoded in the AngularModel
    model.encode(switch_signal, dim=0)
    # If we are not at the end of the temporal window:
    if t_index < tau:
        t_index += 1
    # If we are at the end of the temporal window:
    else:
        # Print the final circuit
        print(model.circ)
        # Measure the module
        result = model.measure()
        # Output the result with print or in a log file
        print("result =", result)
        # Reinitialize the model to encode a new temporal window
        t_index = 1
        model.clear()

    # Show the frame to the user
    cv2.imshow("Camera", frame)

    # Get the user input (press 'q' to quit)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
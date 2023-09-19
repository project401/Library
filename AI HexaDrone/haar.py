pi@raspberrypi:~ $ cat haar.py
import cv2

from dronekit import connect, VehicleMode, LocationGlobalRelative


# Set up option parsing to get connection string


# Connect to the Vehicle
print('Connecting to vehicle on: ')
vehicle = connect("/dev/ttyACM0", wait_ready=False ,baud = 57600)

#vehicle.mode = VehicleMode("GUIDED")

# Load the cascade
face_cascade = cv2.CascadeClassifier("opencv/data/haarcascades/haarcascade_smile.xml")

# Open camera
cap = cv2.VideoCapture(0)

# Read until video is completed
while True:
    # Read a frame from the video capture device
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the grayscale image
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)

    # If a face is detected, armd the vehicle.
    if len(faces) > 0:
        vehicle.armed=True
        break

    # Display the frame with face rectangles
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

    # Show the frame on the screen
    cv2.imshow('frame', frame)

    # Exit the loop if the "q" key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture device and close all windows
cap.release()
cv2.destroyAllWindows()

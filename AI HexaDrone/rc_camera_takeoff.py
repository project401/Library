rc_camera_takeoff.py
from __future__ import print_function
import time
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative




# Connect to the Vehicle
print('Connecting to vehicle on: ')
vehicle = connect("/dev/ttyACM0", wait_ready=False ,baud = 57600)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    counter = 0

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        # if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95 :
        #     print("Reached target altitude")
        #     break
        if counter >= 5:
            break
        time.sleep(1)
        counter += 1





arm_and_takeoff(3)

# Load the cascade
face_cascade = cv2.CascadeClassifier("opencv/data/haarcascades/haarcascade_fullbody.xml")

# Open camera
cap = cv2.VideoCapture(0)

# Read until video is completed
while True:
    # Read a frame from the video capture device
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the grayscale image
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=2)

    # If a face is detected, change the mode of the drone to "AUTO"

    # Display the frame with face rectangles
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

    # Show the frame on the screen
    cv2.imshow('frame', frame)

    # Exit the loop if the "q" key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

time.sleep(20)


print("Returning to LAND")
vehicle.mode = VehicleMode("LAND")


# Release the video capture device and close all windows
cap.release()
cv2.destroyAllWindows()

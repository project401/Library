Rc_TrackFixed.py
from __future__ import print_function
import time
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


# Connect to the Vehicle
print('Connecting to vehicle on: ')
vehicle = connect("/dev/ttyACM0", wait_ready=False ,baud = 57600)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    counter = 0

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if counter >= 5:
            break
        time.sleep(1)
        counter += 1




def send_movement_command_YAW(heading):
    global vehicle
    speed = 0
    direction = 1 #direction -1 ccw, 1 cw

    #heading 0 to 360 degree. if negative then ccw

    print("Sending YAW movement command with heading: %f" % heading)

    if heading < 0:
        heading = heading*-1
        direction = -1

    #point drone into correct heading
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,
        speed,      #speed deg/s
        direction,
        1,          #relative offset 1
        0, 0, 0)

    # send command to vehicle
    vehicle.send_mavlink(msg)
    #Vehicle.commands.flush()

def track_person():
    while True:
        # Read a frame from the video capture device
        ret, frame = cap.read()

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect bodies in the grayscale image
        bodies = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=2)

        # If a body is detected, change the mode of the drone to "GUIDED"
        if len(bodies) > 0:
            vehicle.mode = VehicleMode("GUIDED")
            # calculate the centroid of the first body detected
            x, y, w, h = bodies[0]
            center_x = x + w // 2

            # calculate the direction the drone needs to rotate
            # we only need to consider the x-direction for yaw
            direction = center_x - frame.shape[1] // 2

            # Scale direction to a valid heading degree.
            # This is just a placeholder. You need to replace this with the actual scaling.
            heading = direction * 0.5  # placeholder for scaling

            # Send the yaw command
            send_movement_command_YAW(heading)

        # Display the frame with body rectangles
        for (x, y, w, h) in bodies:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Show the frame on the screen
        cv2.imshow('frame', frame)

        # Exit the loop if the "q" key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


arm_and_takeoff(3)
face_cascade = cv2.CascadeClassifier("opencv/data/haarcascades/haarcascade_fullbody.xml")
cap = cv2.VideoCapture(0)
track_person()
time.sleep(200)
print("Returning to LAND")
vehicle.mode = VehicleMode("LAND")
cap.release()
cv2.destroyAllWindows()

RC_FOLLOW.py
from __future__ import print_function
import time
import cv2
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# Connect to the Vehicle
print('Connecting to vehicle...')
vehicle = connect("/dev/ttyACM0", wait_ready=False, baud=57600)

# PID controller constants
KP = 0.5   # Proportional gain
KI = 0.1   # Integral gain
KD = 0.2   # Derivative gain

# Variables for PID controller
prev_error = 0
integral = 0

# Maximum yaw rate in deg/s
MAX_YAW_RATE = 10
TARGET_ALTITUDE = 10

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and flies to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def send_movement_command_YAW(heading):
    global vehicle
    speed = 10  # Adjust this value to control the yaw speed

    # PID controller variables
    global prev_error
    global integral

    print("Sending YAW movement command with heading: %f" % heading)

    error = heading
    integral += error
    derivative = error - prev_error

    # Calculate the control signal using PID formula
    pid_output = KP * error + KI * integral + KD * derivative

    # Limit the maximum yaw rate
    pid_output = min(max(pid_output, -MAX_YAW_RATE), MAX_YAW_RATE)

    direction = 1 if pid_output >= 0 else -1

    # Point drone into the correct heading
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        abs(pid_output),
        speed,  # Speed deg/s
        direction,
        1,  # Relative offset 1
        0, 0, 0)

    # Send command to vehicle
    vehicle.send_mavlink(msg)

    # Update previous error for next iteration
    prev_error = error

def track_person():
    # Create the Cascade Classifier object
    body_cascade = cv2.CascadeClassifier("opencv/data/haarcascades/haarcascade_fullbody.xml")
    while True:
        # Read a frame from the video capture device
        ret, frame = cap.read()

        # Convert the frame to grayscalea
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect bodies in the grayscale image
        bodies = body_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

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
            heading = direction * 1.0  # placeholder for scaling

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

def send_movement_command_XYA(velocity_x, velocity_y, altitude):
    global vehicle

    #velocity_x positive = forward. negative = backwards
    #velocity_y positive = right. negative = left
    #velocity_z positive = down. negative = up (Yes really!)

    print("Sending XYZ movement command with v_x(forward/backward): %f v_y(right/left): %f " % (velocity_x,velocity_y))

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,  #relative to drone heading pos relative to EKF origin
        0b0000111111100011, #ignore velocity z and other pos arguments
        0, 0, altitude,
        velocity_x, velocity_y, 0,
        0, 0, 0,
        0, 0)

    vehicle.send_mavlink(msg)
    #Vehicle.commands.flush()

def land_vehicle():
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print("Waiting for landing...")
        time.sleep(1)
    print("Landed and motors are disarmed.")



def track_and_follow_person():

    while True:

        # Read a frame from the video capture device
        ret, frame = cap.read()

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect bodies in the grayscale image
        bodies = body_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

        # If a body is detected, follow it
        if len(bodies) > 0:
            vehicle.mode = VehicleMode("GUIDED")
            # For simplicity, follow the first detected body
            body_x, body_y, body_w, body_h = bodies[0]
            center_x = body_x + body_w / 2
            center_y = body_y + body_h / 2
            frame_center_x = frame.shape[1] / 2
            frame_center_y = frame.shape[0] / 2
            error_x = center_x - frame_center_x
            error_y = center_y - frame_center_y

            # Speed needs to be adjusted according to your requirements
            speed_factor = 0.01
            velocity_x = speed_factor * error_x
            velocity_y = speed_factor * error_y

            send_movement_command_XYA(velocity_x, velocity_y, TARGET_ALTITUDE)
            print(velocity_x, velocity_y, TARGET_ALTITUDE)

        # If no body is detected, maintain the current altitude
        else:
            vehicle.simple_goto(vehicle.location.global_relative_frame, groundspeed=0)




arm_and_takeoff(5)
body_cascade = cv2.CascadeClassifier("opencv/data/haarcascades/haarcascade_fullbody.xml")
bodies = cv2.CascadeClassifier("opencv/data/haarcascades/haarcascade_fullbody.xml")

cap = cv2.VideoCapture(0)
track_person()
track_and_follow_person()
cap.release()
cv2.destroyAllWindows()

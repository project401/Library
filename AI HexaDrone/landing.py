landing.py
from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative


# Set up option parsing to get connection string


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





#arm_and_takeoff(2)



#print("wait 5 sec")
# sleep so we can see the change in map
#time.sleep(5)


print("Returning to LAND")
vehicle.mode = VehicleMode("LAND")
time.sleep(3)
# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

from dronekit import connect, VehicleMode
import time

vehicle = connect(ip = "/dev/ttyAMA0", baud=57600)


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
vehicle.mode = VehicleMode("AUTO")
vehicle.armed = True


# Confirm vehicle armed before attempting to take off
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

vehicle.close()


# print("Changing Vehicle Mode...")

# vehicle.mode = VehicleMode("STABILIZE")
# while vehicle.mode != "STABILIZE":
#     print("Waiting on vehicle to enter STABILIZE")
#     time.sleep(2)

# print("Changing Vehicle Mode...")

# vehicle.mode = VehicleMode("GUIDED")
# while vehicle.mode != "GUIDED":
#     print("Waiting on vehicle to enter GUIDED")
#     time.sleep(2)

print("Dang you did it.")
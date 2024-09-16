from dronekit import connect, VehicleMode
import time

vehicle = connect(ip = "/dev/ttyAMA0", baud=57600)

print("Changing Vehicle Mode...")

vehicle.mode = VehicleMode("STABILIZE")
while vehicle.mode != "STABILIZE":
    print("Waiting on vehicle to enter STABILIZE")
    time.sleep(2)

print("Changing Vehicle Mode...")

vehicle.mode = VehicleMode("GUIDED")
while vehicle.mode != "GUIDED":
    print("Waiting on vehicle to enter GUIDED")
    time.sleep(2)

print("Dang you did it.")
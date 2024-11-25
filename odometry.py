import time
import math

# This class is used to store odometry information to a text file.
class Odometer(object):
    def __init__(self, vehicle) -> None:
        # Last positions for calculating displacement every update
        self.last_north = 0
        self.last_east = 0
        # last time for calculating time duration every update.
        self.last_time = time.time()

        self.odometry_list = []  # list to hold odometry data [duration, displacement]

        self.vehicle = vehicle  # Vehicle object to pull positions from

    def update(self) -> None:
        new_odometry_measurement = []
        # Calculate dt
        current_time = time.time()
        dt = current_time - self.last_time
        # Calculate displacement
        displacement = self.displacement_finder()

        # set current position to last position
        self.last_north = self.vehicle.location.local_frame.north
        self.last_east = self.vehicle.location.local_frame.east
        self.last_down = self.vehicle.location.local_frame.down

        # append to list of measurements
        self.odometry_list.append([dt, displacement])

    # Finds displacement between last position and current position
    def displacement_finder(self) -> float:
        return abs(
            math.sqrt(
                (self.vehicle.location.local_frame.north - self.last_north) ** 2
                + (self.vehicle.location.local_frame.east - self.last_east) ** 2
                + (self.vehicle.location.local_frame.down - self.last_down) ** 2
            )
        )

    # Writes contents of the odometry list to a file
    def write(self) -> float:
        with open("Odometry/odometer_measurements.txt", "w") as f:
            for measurement in self.odometry_list:
                f.write(f"{line}\n")
        f.close()

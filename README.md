# DroNS-3: Simplifying Networked Drone Applications
DroNS-3 is a software framework that features an easy-to-use autopilot designed to simplify the simulation-to-hardware life cycle of drone research. DroNS-3 is tailored toward drone networking and data collection applications but can also be used for more general applications.

DroNS-3 utilizes the [Ardupilot SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) drone simulator and the [NS-3 networking simulator](https://www.nsnam.org/) together. The DroNS-3 autopilot is built on top of the [DroneKit](https://github.com/dronekit) autopilot using [MAVLink](https://mavlink.io/en/). For more information on the framework's design, please refer to the [DroNS-3 publication](https://dl.acm.org/doi/10.1145/3597060.3597239).

# Using the framework

## Simulation
To set up and run DroNS-3 simulation, please follow the instructions given in the [Wiki](https://github.com/pervasive-computing-systems-group/DroNS3/wiki).

## Deploying On Physical Drones
To run the DroNS-3 autopilot on a physical drone, you must write a version of the [mission_wrapper.py script](https://github.com/pervasive-computing-systems-group/DroNS3/blob/main/mission_wrapper.py). We are currently working on creating an easy-to-use script to help facilitate this process. An example of how to write your own script can be found in an older version of DroNS-3 found in the [Minimizing Data Latency](https://github.com/JonD07/MinLatencyWSN) project.

# Contributing
We welcome new contributors and are hoping to continue development on the framework. If you would like to get involved please reach out to [Jonathan Diller](https://jond07.github.io/).

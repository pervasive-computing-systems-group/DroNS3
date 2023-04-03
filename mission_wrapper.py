import time
import missions
import commands
from dronekit import connect, VehicleMode, LocationGlobalRelative
import subprocess as sb
import sys
import defines
import signal
import os


class Process_Thread_Holder(object):

    def __init__(self) -> None:
        self.processes = []
        self.threads = []


    def add_process(self, process) -> int:
        self.processes.append(process)
    
    def add_thread(self, thread) -> int:
        self.threads.append(thread)

class Wrapper(object):

    def __init__(self) -> None:

        self.pth = Process_Thread_Holder()
        
        self.debug = defines.debug
        missions.setSimulation(True)
        commands.setHolder(self.pth)
        missions.setHolder(self.pth)

        seed = 0
        # path = defines.MISSION_PATH
        missions.setSeed(seed)

        print("Connect to simulation vehicle")
        self.vehicle = connect('localhost:14550', wait_ready=True)

        # Get some vehicle attributes (state)
        print("Contacted vehicle!")

        # Pass the vehicle to rest of ICCS autopilot
        missions.pass_vehicle(self.vehicle)

        # Get Vehicle Home location - will be `None` until first set by autopilot
        while not self.vehicle.home_location:
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()
            if not self.vehicle.home_location:
                print(" Waiting for home location ...")

        # We have a home location
        print(" Home location: %s" % self.vehicle.home_location)
        print(self.vehicle.parameters.keys)
        # for key, value in zip(self.vehicle.parameters.keys, self.vehicle.parameters.values):
        #     print (" Key:%s Value:%s" % (key,value))

        # Program-specific constants and configuration
        self.vehicle.airspeed = 10 # m/s
        self.vehicle.groundspeed = 10 # m/s

        self.state = {
            'mission': None,
            'is_disabled': False
        }

        signal.signal(signal.SIGINT, self.signal_handler)

    def start_mission(self, mission=None):
        if issubclass(type(self.state['mission']), missions.Mission):
            print("Terminating mission", self.state['mission'].name)
            self.state['mission'].dispose()
            while self.state['mission'].thread.is_alive():
                time.sleep(0.05)
        self.state['mission'] = mission
        if mission == None:
            self.state['mission'] = missions.General(self.vehicle)
        print("Starting mission", self.state['mission'].name)
        # For simulation, set GUIDED mode and arm before starting a mission
        self.guided_and_arm()
        self.state['mission'].start()
        self.vehicle.close()

    def guided_and_arm(self):
        print("Set Mode to GUIDED")
        self.vehicle.mode = VehicleMode("GUIDED")
        print("Arming motors")
        self.vehicle.armed = True
        while not self.vehicle.mode.name=='GUIDED' and not self.vehicle.armed:
            print(" Getting ready to take off ...")
            time.sleep(1)

    def signal_handler(self, signum, frame):
        if(self.debug):
            print("Ending processes . . .")

        self.state['mission'].terminate = True
        for p_index in range(len(self.pth.processes)):
            try:
                #os.killpg(os.getpgid(self.pth.processes.pid), signal.SIGTERM)
                self.pth.processes[p_index].kill()
                
                while self.pth.processes[p_index].poll() is None:
                    if self.debug:
                        print("Process {i} still running with return code {rc}, terminating again . . .".format(rc = self.pth.processes[p_index].poll(), i = p_index))
                    try:
                        self.pth.processes[p_index].kill()
                    
                    except Exception:
                        if self.debug:
                            print("Terminating process failed . . . ")

                if self.debug:
                    print("Terminated process {i} with return code {rc}".format(i = p_index, rc = self.pth.processes[p_index].poll()))
            except Exception:
                if self.debug:
                    print("Process {i} failed to terminate. The process likely ended. Return code {rc}".format(i = p_index, rc = self.pth.processes[p_index].poll()))
                          
        if self.debug:
            print("Terminating . . . ")
        self.vehicle.close()
        sys.exit(1)
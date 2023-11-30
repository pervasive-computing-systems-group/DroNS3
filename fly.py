from dronekit import connect, VehicleMode
from pymavlink import mavutil
import threading
import time
import subprocess as sb

import missions
import commands
from mission_wrapper import Wrapper
import os
from datetime import date
from argparse import ArgumentParser
#import config_copter


def create_logfile(plan):
    global log_file
    global start_time
    global plan_file

    plan_file = plan

    plan = plan.split(".")[0]

    if not os.path.exists("logs"):
        os.mkdir("logs")

    d = date.today().strftime("%b_%d_%Y")

    if os.path.exists("logs/"+str(plan)+"_" +str(d)+".txt"):
        m = 1

        while os.path.exists("logs/"+str(plan)+ "_" + str(d)+ "_" + str(m)+ ".txt"):
            m += 1

        d += "_"+str(m)
    log_file = "logs/"+str(plan)+"_"+str(d)+".txt"
    start_time = time.time()
    with open(log_file, "w") as f:
        f.write("voltage,current,level,time\n")
    

class Voltage_Check(commands.Command):

    def __init__(self, vehicle, mission, args, debug = False):
        self.vehicle = vehicle
        self.mission = mission
        self.args = args
        self.done = False
        self.debug = debug
        self.log_file = log_file
        


    def begin(self):
        battery = self.vehicle.battery
        with open(self.log_file, "a") if os.path.exists(self.log_file) else open(self.log_file, "w") as f:
            f.write(str(battery.voltage)+ "," + str(battery.current)+","+str(battery.level)+","+str(time.time()-start_time) + "\n")
        self.done = True

    def is_done(self):
        return self.done


def init(sim = False):
    global state
    global vehicle
    global wrapper



    print("Initializing vehicle")
    
    wrapper = Wrapper()
    vehicle = wrapper.vehicle
    

    state = {
        'mission': None,
        'channels': wrapper.vehicle.channels,
        'is_disabled': False,
        'is_aodv': False
    }

    if sim:
        wrapper.start_mission(mission=missions.General(wrapper.vehicle, mission_file=plan_file, debug = True, custom_commands=[Voltage_Check], is_sim=True))
    else:
        wrapper.vehicle.add_attribute_listener('channels', start)

def start(self, name, channels):
    # Conditions to start or resume a mission
    if channels['3'] < 988:
        if vehicle.armed:
            if not state['is_disabled']:
                print("Disabling missions...")
                state['mission'].dispose()
                state['is_disabled'] = True
        else:
            if state['mission'] != "PREARM":
                if issubclass(type(state['mission']), missions.Mission):
                    state['mission'].dispose()
                print("Waiting to become armable...")
                state['mission'] = "PREARM"
        state['channels'] = channels.copy()
        return
    else:
        if vehicle.armed:
            if state['is_disabled']:
                print("Re-enabling missions...")
                state['is_disabled'] = False

    # Select mission
    if state['channels']['3'] >= 988: # Makes sure changes aren't picked up from turning on the RC

        # If Switch A changes
        if abs(channels['5'] - state['channels']['5']) > 100:
            wrapper.start_mission(mission=missions.General(wrapper.vehicle, mission_file="laps.pln", debug = True, custom_commands=[Voltage_Check], is_sim=True))
        # Ready to start mission
        elif not state['mission'] == "READY" and (state['mission'] == None or state['mission'] == "PREARM"): 
            print("Ready for takeoff! Select mission...")
            state['mission'] = "READY"

        # If Switch B is active
        # if channels['6'] > 1100:
        #     state['is_aodv'] = True
        # else:
        #     state['is_aodv'] = False
            
        # #If the third switch is flipped, start test mission.
        # if abs(channels['7'] - state['channels']['7']) > 100:
        #     state['mission'].dispose()
        #     vehicle.mode = VehicleMode('STABILIZE')
           
        

    # Hold onto previous channel values
    state['channels'] = channels.copy()

# def start_next_mission(mission=None):
#     if issubclass(type(state['mission']), missions.Mission):
#         print("Terminating mission", state['mission'].name)
#         state['mission'].dispose()
#         while state['mission'].thread.is_alive():
#             time.sleep(0.05)
#     state['mission'] = mission
#     print("Starting mission", state['mission'].name)
#     state['mission'].start()

if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("plan", help="Plan file for general mission")
    parser.add_argument("-s", "--sim", action="store_true", help="Flag for if simulation is being used (removes controller channel components)")

    args = parser.parse_args()

    create_logfile(args.plan)
    init(sim = True)
    while True:
        time.sleep(1)
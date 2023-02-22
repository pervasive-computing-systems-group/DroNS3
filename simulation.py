import time
import missions
import commands
from dronekit import connect, VehicleMode, LocationGlobalRelative
import subprocess as sb
import sys
# 
# To run the simulation, open a terminal in ardupilot/ArduCopter and run:
# sim_vehicle.py -f quad -L CSM_SurveyField --console --map --osd
# 

## For simulation
#-- Define the function for takeoff
def guided_and_arm():
	print("Set Mode to GUIDED")
	vehicle.mode = VehicleMode("GUIDED")
	print("Arming motors")
	vehicle.armed = True
	while not vehicle.mode.name=='GUIDED' and not vehicle.armed:
		print(" Getting ready to take off ...")
		time.sleep(1)


def init():
	# We are not running the simulation
	missions.setSimulation(True)
	commands.setSimulation(True)

	global state
	global vehicle

	print("Connect to simulation vehicle")
	vehicle = connect('localhost:14550', wait_ready=True)

	# Get some vehicle attributes (state)
	print("Contacted vehicle!")

	# Pass the vehicle to rest of ICCS autopilot
	missions.pass_vehicle(vehicle)
	commands.pass_vehicle(vehicle)

	# Get Vehicle Home location - will be `None` until first set by autopilot
	while not vehicle.home_location:
		cmds = vehicle.commands
		cmds.download()
		cmds.wait_ready()
		if not vehicle.home_location:
			print(" Waiting for home location ...")

	# We have a home location
	print(" Home location: %s" % vehicle.home_location)

	# Program-specific constants and configuration
	vehicle.airspeed = 10 # m/s
	vehicle.groundspeed = 10 # m/s

	state = {
		'mission': None,
		'is_disabled': False
	}


def start_next_mission(mission=None):
	if issubclass(type(state['mission']), missions.Mission):
		print("Terminating mission", state['mission'].name)
		state['mission'].dispose()
		while state['mission'].thread.is_alive():
			time.sleep(0.05)
	state['mission'] = mission
	print("Starting mission", state['mission'].name)
	# For simulation, set GUIDED mode and arm before starting a mission
	guided_and_arm()
	state['mission'].start()

def run_sim(iterations, start_seed):
	for i in range(start_seed, start_seed + iterations):
		missions.setSeed(i)
		init()
		start_next_mission(mission=missions.CollectWSNData())
		print("Mission Normal Completed")
		# init()
		# start_next_mission(mission=missions.CollectWSNDataNaive())
		# print("Mission 2 Completed")
		init()
		start_next_mission(mission=missions.CollectWSNDataNoSub())
		print("Mission NOSUB Completed")


if __name__ == '__main__':

	#for i in range(start_seed, start_seed + iterations):
	seed = int(sys.argv[1])
	path = sys.argv[2]
	missions.setSeed(seed)
	missions.setPath(path)
	init()
	mission_num = int(sys.argv[3])
	if(mission_num == 0):
		start_next_mission(mission=missions.CollectWSNData())
		print("Mission Normal Completed")

	elif(mission_num == 1):
		start_next_mission(mission=missions.CollectWSNDataNaive())
		print("Mission Normal Completed")

	elif(mission_num == 2):
		start_next_mission(mission=missions.CollectWSNDataLKH())
		print("Mission LKH Completed")
		
	elif(mission_num == 3):
		start_next_mission(mission=missions.CollectWSNDataNoSub())
		print("Mission NOSUB Completed")

	sys.exit(0)

	# sim_process = sb.Popen("/home/ardupilot/ArduCopter/sim_vehicle.py -f quad -L CSM_SurveyField --console --map --osd")
	# control_thread = Thread(target = run_sim(0,20))


	# while True:
	# 	if control_thread.is_alive():
	# 		pass
	# 	else:
	# 		print("Control thread failed. Starting new execution")
	# 		sim_thread
	
	

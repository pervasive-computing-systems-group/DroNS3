import abc
from dronekit import VehicleMode
import threading
import queue
import commands
import subprocess as sb
from collections import deque
import time
import numpy as np
import defines
import math
import signal
from solver import LKH_Solver
from telemetry import WSNData
import telemetry
import energy_budget
'''
TODO:

In general replace all global variables with arguments or objects.
For vehicle, if no argument is passed, initialize the vehicle (look at init() from simulation) in the parent Mission class
Make all file paths passed rather than hardcoded (maybe exception for LKH stuff or any other library)
Make general mission that uses a file with a series of commands in it.
Remove all NS3 calls.
Create a class that simulates the network communication stuff (The NS3 Stuff).
'''

# ph = ProcessHandler(debug=defines.debug)

# def signal_handler(signum, frame):
# 		ph.signal_handler(signum, frame)
# 		exit(1)


# signal.signal(signal.SIGINT, signal_handler)

#TODO: Remove reliance on global variables
def setSimulation(sim):
	global running_sim
	running_sim = sim
	# if running_sim:
	# 	import numpy as np

def setHolder(holder_t):
	global holder 
	holder = holder_t

def setSeed(vseed):
	global seed
	seed = vseed

def setPath(vpath):
	global path
	path = vpath


#TODO: Refactor to be passed into the mission
def pass_vehicle(passed_vehicle):
	global vehicle
	vehicle = passed_vehicle

class Mission(object):
	__metaclass__ = abc.ABCMeta
	terminate = False
	name = "Name not set"
	thread = None
	q = deque()

	#TODO: Refactor to include optional global arguments - simulation + any others we want
	@abc.abstractmethod
	def __init__(self):
		pass

	def start(self):
		self.thread = threading.Thread(target=self.update_wrapper, name=self.name)
		self.thread.start()
		if running_sim:
			# Wait here for the thread to re-join
			self.thread.join()

	# Wrapper function for updating mission if not terminated
	def update_wrapper(self):
		self.command.begin()
		
		#This assumes we always go to LAND - we may need some other signal to trigger the mission to pause or stop
		#TODO: more robust failsafe
		while not self.terminate:
			if vehicle.mode == VehicleMode("GUIDED") and (vehicle.system_status == "ACTIVE" or vehicle.system_status == "STANDBY"):
				self.update()
			else:
				time.sleep(0.1)
				print(vehicle.mode, vehicle.system_status)
				print("Vehicle no longer in guided or in non-stable flight mode")

	# Periodically called to check command status/is-done
	def update(self):
		# Check current command
		if self.command.is_done():
			# Current command finished, check if there is another command
			if self.q:
				# Run next command
				self.command = self.q.popleft()
				self.command.begin()
			else:
				# deque is empty
				self.dispose()
		# Current command isn't complete, call update on command
		else:
			#  Command not complete, call update
			self.command.update()

	def dispose(self):
		self.terminate = True

	

#TODO: Rename? Name not intuitive
class Manual(Mission):
	name = "MANUAL"

	def __init__(self):
		self.is_loiter = False
		self.command_idle = commands.Idle('LOITER', vehicle)
		self.command_idle.can_idle = False
		self.command = commands.GainAlt(2, vehicle)
		self.q.append(self.command_idle)

	def update(self):
		if vehicle.channels['3'] > 1500 and not self.is_loiter:
			self.is_loiter = True
			self.command_idle.can_idle = True
		super(Manual, self).update()




class CollectWSNData(Mission):

	'''
		Mission to collect data from a series of WSN nodes. 

		plan_path: path to drone plan file. Default value is "drone_plan.pln".
			The drone plan file utilizes a series of commands, with one command per line. The available commands are:
				0: Fly to waypoint relative to home location
					0 [relative x] [relative y] [relative altitude]
				1: Connect to WSN node
					1 [node number] [communication power]
				2: Set mission altitude
					2 [mission altitude]

		node_path: path to node info file. Default value is "node_info.txt"
			The node info file lists the nodes in the WSN. Each line contains one node. Coordinates are relative to home locations.
				[node number] [IP address] [sensor data size] [relative x] [relative y]

		output_path: path to directory where results will be stored. Default value is "" and will write to the working directory

		algorithm: Chooses which algorithm to use. Default value is "DEFAULT"
			Available algorithms:
				DEFAULT: The default configuration. Any time the drone cannot connect to a node, it moves toward the node until it connects. If there are
					multiple nodes that did not connect at a single stop, it will use the order in the drone plan file.
				NAIVE: Alters the DEFAULT algorithm by not stopping when it connects, but always stopping directly on top of the node.
				ONLINE: Alters the DEFAULT algorithm by ordering the subtours via the LKH TSP solver algorithm. (Currently not working ://)
				NO_SUB: Alters the DEFAULT algorithm by not executing subtours.

	'''
	name = "WSN_DATA"
	missed_q = deque()

	CMD_WAYPOINT = 0
	CMD_COLL_DATA = 1
	CMD_MSN_ALT = 2
	start_time = 0
	end_time = 0
	

	def __init__(self, plan_path = "sample_wsn_mission/drone_plan.pln", node_path = "sample_wsn_mission/node_info.txt", output_path = "./", algorithm = "DEFAULT",  speed = 10, b_rate = 72, v_bat = 15, pm = 50, ph = 40):
		#self.sm = b_rate * v_bat / pm # this will be total amount of time drone can travel at max speed
		#self.sh = b_rate * v_bat / pm # this will be total amount of time drone can hover - both of these will be fixed and provided
		self.sm = 750 #750 is what it probably should be based on jonathan's code for other thing 
		self.sh = 14*60 #14*60 based on jonathan code
		self.energy_budget = self.sm + self.sh #energy budget: time that drone can hover + move
		self.initial_budget = self.energy_budget
		self.budget_percent = energy_budget.EnergyBudget(self)
		self.data = WSNData()
		self.speed = speed
		self.plan_path = plan_path
		self.algorithm = algorithm
		self.output_path = output_path
		self.node_path = node_path
		# Set random seed for consistancy
		np.random.seed(seed)
		self.mission_alt = 50
		# Add take-off command
		self.q.append(commands.GainAlt(self.mission_alt, vehicle))
		self.energy_budget -= self.mission_alt/speed
		# Add Timer-Start command
		self.q.append(commands.StartTimer())
		# Get list of commands from file
		file1 = open(self.plan_path,"r+")
		# Node power-settings
		self.nPowers = {-1: 0}
		# Run through each command in list
		for aline in file1:
			values = aline.split()
			# If cmd = 0 (waypoint)
			if int(values[0]) == self.CMD_WAYPOINT:
				# Add waypoint movement to queue
				self.q.append(commands.MoveToWaypoint(float(values[1]), float(values[2]), float(values[3]), vehicle))
			# else if cmd = 1 (data collection)
			elif int(values[0]) == self.CMD_COLL_DATA:
				print(values)
				# TODO: Add a normal-distribution for the nodes range
				arr = np.random.normal(float(values[2]) - 1, 8, 1)
				self.nPowers[int(values[1])] = arr[0]
				# Add collect data command
				self.q.append(commands.CollectData(int(values[1]), arr[0], vehicle, node_path, self.data, sim = running_sim))
			elif int(values[0]) == self.CMD_MSN_ALT:
				# Set mission altitude
				self.mission_alt = float(values[1])
			else:
				print("ERROR: unexpected cmd in pln file")
		file1.close()
		# Add Return-To-Home command
		self.q.append(commands.ReturnHome(self.mission_alt, vehicle))
		# Add Timer-Stop command
		self.q.append(commands.StopTimer())
		if running_sim:
			# Add land command
			self.q.append(commands.Land(vehicle))
		# Add first command as current command
		self.command = self.q.popleft()
		print("Node power settings")
		print(self.nPowers)

	def pop_loop(self):
		return_dist = commands.ReturnHome.distance_finder(self.command)
		cost_to_home = self.cost + self.mission_alt/self.speed  + return_dist/self.speed
		print("cost to home", cost_to_home)
		while (cost_to_home) > self.energy_budget: 
				if len(self.q) > 3:
					self.q.popleft()
					if self.command != commands.CollectData:
						if self.command == commands.MoveToWaypoint:
							self.cost = commands.MoveToWaypoint.distance_finder(self.command)/self.speed
						elif self.command == commands.MoveAndCollectData:
							self.cost = commands.MoveAndCollectData.distance_finder(self.command)/self.speed
					else:
						self.cost = commands.CollectData.distance_finder(self.command)/self.speed + self.sm/self.sh #estimate
					cost_to_home = self.cost + self.mission_alt/self.speed + return_dist/self.speed
				else:
					break
	def update(self):
		if isinstance(self.command, commands.CollectData):
			self.cost = self.command.cost/self.speed
			# Check if we are done collecting data
			if self.command.is_done():
				print(self.command.time)
				self.pop_loop()
				self.energy_budget -= self.cost
				self.energy_budget -= self.command.time * (self.sm/self.sh)
				print(self.energy_budget)
				# Finished, check is collection was successful
				if self.command.collection_success():
					print("Total collected data: " + str(self.data.data_collected))
				else:
					print("Total collected data: " + str(self.data.data_collected))
					# print("Failed collected data from " + str(self.command.node_ID))
					# Add this node to missed nodes queue
					self.missed_q.append(self.command.node_ID)
					# Check if we are done collecting data at the hovering location
				if not isinstance(self.q[0], commands.CollectData):
					if self.algorithm == "ONLINE":
						hpp_points = []
						hpp_points_id = []
						while self.missed_q:
							n = self.missed_q.pop()
							hpp_points_id.append(n)
							file1 = open(self.node_path,"r+")
							for aline in file1:
								values = aline.split()
								if int(values[0]) == n:
									east = float(values[3])
									north = float(values[4])
							file1.close()
							# Add moving-collecting from this node to the command queue
							hpp_points.append([east,north])
						if isinstance(self.q[0], commands.MoveToWaypoint):
							hpp_points.append([self.q[0].east, self.q[0].north])
							hpp_points_id.append(-1)
							hpp_points.append(commands.get_xy(vehicle))
							hpp_points_id.append(-1)
							
						else:
							hpp_points.append([self.q[0].east, self.q[0].north])
							hpp_points_id.append(-1)

						print(hpp_points)

						if len(hpp_points) < 3:
							for p, i in zip(hpp_points, hpp_points_id):
								if i == -1:
									self.q.appendleft(commands.MoveToWaypoint(p[0], p[1], self.mission_alt, vehicle))
								else:
									self.q.appendleft(commands.MoveAndCollectData(i, self.mission_alt, self.nPowers[i], vehicle, self.data, node_path = self.node_path, sim = running_sim))
						else:
							lkh = LKH_Solver([hpp_points, len(hpp_points) - 2, len(hpp_points) - 1, holder])
							LKH_path = lkh.solve()
							for p in LKH_path:
								if hpp_points_id[p] == -1:
									self.q.appendleft(commands.MoveToWaypoint(hpp_points[p][0], hpp_points[p][1], self.mission_alt, vehicle))
								else:
									self.q.appendleft(commands.MoveAndCollectData(hpp_points_id[p], self.mission_alt, self.nPowers[p], vehicle, self.data, node_path = self.node_path, sim = running_sim))

					# Done collecting data at this point, start recovery
					while self.missed_q:
						n = self.missed_q.pop()
						if self.algorithm != "NO_SUB":
							print("Added move-collect command")
								# Add moving-collecting from this node to the command queue
							self.q.appendleft(commands.MoveAndCollectData(n, self.mission_alt, self.nPowers[n], vehicle, self.data, algorithm = "NAIVE" if self.algorithm == "NAIVE" else "Default", node_path = self.node_path, sim = running_sim))
							
		# Check to see if we just finished a move-collect command
		
		if isinstance(self.command, commands.MoveAndCollectData):
			self.cost = self.command.cost/self.speed
			# Check if we are done collecting data
			if self.command.is_done():
				self.pop_loop()
				self.energy_budget -= self.cost
				self.energy_budget -= self.command.time * (self.sm/self.sh)

		
			# Check if this was the last move-collect command
			if not isinstance(self.q[0], commands.MoveAndCollectData):
				# TODO: Update the next waypoint
				pass

		if isinstance(self.command, commands.MoveToWaypoint):
			self.cost = self.command.cost/self.speed
			# Check if we are done collecting data
			if self.command.is_done():
				self.pop_loop()
				self.energy_budget -= self.cost

		if isinstance(self.command, commands.ReturnHome):
			self.cost = self.command.cost/self.speed
			self.energy_budget -= self.cost

		if isinstance(self.command, commands.Land):
			self.cost = self.mission_alt/self.speed
			self.energy_budget -= self.cost


		
		if isinstance(self.command, commands.StartTimer):
			self.data.start_timer()
			print("Starting Timer")

		if isinstance(self.command, commands.StopTimer):
			self.data.stop_timer()
			print("Stopping Timer")
			e = energy_budget.EnergyBudget(mission = self)
			self.energy_budget -= self.mission_alt/self.speed
			print(e.percentBudget())
			if self.output_path[-1] != "/":
				self.output_path += "/"

			with open(self.output_path + "flight-time.dat", "a") as tfile:
				tfile.write("0 " + str(self.data.total_time) + "\n")

			with open(self.output_path + "data_collected.dat", "a") as dfile:
				dfile.write("0 " + str(self.data.data_collected) + "\n")

			print("Data Collected:", self.data.data_collected)
			self.terminate = True

		super(CollectWSNData, self).update()



class General(Mission):

	'''
		General is a Mission subclass that takes a mission file with one command on each line. Each command consists of an integer representing 
		a command, and the number of required (or optional) parameters deliniated by spaces.

		Commands:
			0: GainAlt - the drone gains target_altitude (float) meters in altitude. 
				0 <target_altitude>
			1: MoveToWaypoint - the drone moves to the specified location with an optional tolerance parameter.
				1 <east> <north> <up> [tolerance]
			2: ReturnHome - the drone returns to its home location at the specified altitude (float).
				2 <altitude>
			3: Land - the drone decreases its altitude until it reaches the ground. There are no other parameters for this command.
				3
			4: Wait - the drone waits a specified time (float).
				4 <wait_time>
			5: Collect Data - TODO Write this.
			6 + : Custom Commands - User specified commands. A list of commands (references to the command classes, not command objects) that inherit from
			commands.Command must be passed into the optional argument custom_commands. These commands must each take a vehicle parameter, a refrence to the mission,
			and a single array object who's elements are the intended parameters. 
			
	'''
	name = "GENERAL"
	def __init__(self, vehicle, mission_file = "sample_wsn_mission/drone_plan.pln", debug = False, custom_commands = None, is_sim = False, telem = None):
		self.vehicle = vehicle
		self.telemetry = telem
		with open(mission_file) as mf:
			command_list = mf.readlines()

		for c in command_list:
			c = c.split()

			if c[0] == "0":
				#Gain_Alt command
				self.q.append(commands.GainAlt(float(c[1]), self.vehicle))
			elif c[0] == "1":
				#MoveToWaypoint
				if len(c) == 4:
					self.q.append(commands.MoveToWaypoint(float(c[1]), float(c[2]), float(c[3]),self.vehicle, debug = debug))
				elif len(c) == 5:
					self.q.append(commands.MoveToWaypoint(float(c[1]), float(c[2]), float(c[3]),self.vehicle, tolerance = float(c[4]), debug = debug))
			elif c[0] == "2":
				#Return home
				self.q.append(commands.ReturnHome(float(c[1]), self.vehicle, debug = debug))
			elif c[0] == "3":
				#Land
				self.q.append(commands.Land(self.vehicle, debug = debug))
			elif c[0] == "4":
				self.q.append(commands.Wait(float(c[1]), self.vehicle, debug=debug))
			elif c[0] == "5":
				if self.telemetry is None:
					self.telemetry = telemetry.NetworkData()
				if(len(c) == 5):
					self.q.append(commands.CollectDataGeneral(float(c[1]), float(c[2]), float(c[3]), float(c[4]),self.vehicle, self.telemetry, sim = is_sim))
				if(len(c) == 6):
					self.q.append(commands.CollectDataGeneral(float(c[1]), float(c[2]), float(c[3]), float(c[4]), self.vehicle, self.telemetry, sim = is_sim, payload = float(c[5])))
				if(len(c) == 7):
					self.q.append(commands.CollectDataGeneral(float(c[1]), float(c[2]), float(c[3]), float(c[4]), self.vehicle, self.telemetry, sim = is_sim, payload = float(c[5]), delay = True if c[6] == "true" else False))
				if(len(c) == 8):
					self.q.append(commands.CollectDataGeneral(float(c[1]), float(c[2]), float(c[3]), float(c[4]), self.vehicle, self.telemetry, sim = is_sim, payload = float(c[5]), delay = True if c[6] == "true" else False, collection_time = float(c[7])))
			elif custom_commands is None:
					if debug:
						print("No custom commands")
			elif int(c[0]) > 5:
				command = custom_commands[int(c[0]) - 6]

				for i in range(1, len(c)):
					c[i] = float(c[i])

				self.q.append(command(self.vehicle, self, c[1:]))
			
		self.command = self.q.popleft()


class ConnectionTests(Mission):
	'''
		Mission to test connectivity from different waypoints in order to optimize paths

		plan_path: path to drone plan file. Default value is "drone_plan.pln".
			The drone plan file utilizes a series of commands, with one command per line. The available commands are:
				0: Fly to waypoint relative to home location
					0 [relative x] [relative y] [relative altitude]
				1: Connect to RPI server at origin
					1 
				2: Set mission altitude
					2 [mission altitude]

		node_path: path to node info file. Default value is "node_info.txt"
			The node info file lists the nodes in the WSN. Each line contains one node. Coordinates are relative to home locations.
				[node number] [IP address] [sensor data size] [relative x] [relative y]

		output_path: path to directory where results will be stored. Default value is "" and will write to the working directory

		algorithm: Chooses which algorithm to use. Default value is "DEFAULT"
			Available algorithms:
				DEFAULT: The default configuration. Any time the drone cannot connect to a node, it moves toward the node until it connects. If there are
					multiple nodes that did not connect at a single stop, it will use the order in the drone plan file.
				NAIVE: Alters the DEFAULT algorithm by not stopping when it connects, but always stopping directly on top of the node.
				ONLINE: Alters the DEFAULT algorithm by ordering the subtours via the LKH TSP solver algorithm. (Currently not working ://)
				NO_SUB: Alters the DEFAULT algorithm by not executing subtours.

	'''
	name = "CONNECTION"
	missed_q = deque()

	CMD_WAYPOINT = 0
	CMD_COLL_DATA = 1
	CMD_MSN_ALT = 2
	start_time = 0
	end_time = 0
	

	def __init__(self, plan_path = "sample_wsn_mission/drone_plan.pln", output_path = "./", algorithm = "DEFAULT",  speed = 10, b_rate = 72, v_bat = 15, pm = 50, ph = 40):
		#self.sm = b_rate * v_bat / pm # this will be total amount of time drone can travel at max speed
		#self.sh = b_rate * v_bat / pm # this will be total amount of time drone can hover - both of these will be fixed and provided
		self.sm = 750 #750 is what it probably should be based on jonathan's code for other thing 
		self.sh = 14*60 #14*60 based on jonathan code
		self.energy_budget = self.sm + self.sh #energy budget: time that drone can hover + move
		self.initial_budget = self.energy_budget
		self.budget_percent = energy_budget.EnergyBudget(self)
		self.data = WSNData()
		self.speed = speed
		self.plan_path = plan_path
		self.algorithm = algorithm
		self.output_path = output_path
		# Set random seed for consistancy
		np.random.seed(seed)
		self.mission_alt = 50
		# Add take-off command
		self.q.append(commands.GainAlt(self.mission_alt, vehicle))
		self.energy_budget -= self.mission_alt/speed
		# Add Timer-Start command
		self.q.append(commands.StartTimer())
		# Get list of commands from file
		file1 = open(self.plan_path,"r+")
		# Node power-settings
		self.nPowers = {-1: 0}
		# Run through each command in list
		for aline in file1:
			values = aline.split()
			# If cmd = 0 (waypoint)
			if int(values[0]) == self.CMD_WAYPOINT:
				# Add waypoint movement to queue
				self.q.append(commands.MoveToWaypoint(float(values[1]), float(values[2]), float(values[3]), vehicle))
			# else if cmd = 1 (data collection)
			elif int(values[0]) == self.CMD_COLL_DATA:
				# Add connect command
				self.q.append(commands.Connect())
			elif int(values[0]) == self.CMD_MSN_ALT:
				# Set mission altitude
				self.mission_alt = float(values[1])
			else:
				print("ERROR: unexpected cmd in pln file")
		file1.close()
		# Add Return-To-Home command
		self.q.append(commands.ReturnHome(self.mission_alt, vehicle))
		# Add Timer-Stop command
		self.q.append(commands.StopTimer())
		if running_sim:
			# Add land command
			self.q.append(commands.Land(vehicle))
		# Add first command as current command
		self.command = self.q.popleft()
		print("Node power settings")
		print(self.nPowers)

	def pop_loop(self):
		return_dist = commands.ReturnHome.distance_finder(self.command)
		cost_to_home = self.cost + self.mission_alt/self.speed  + return_dist/self.speed
		print("cost to home", cost_to_home)
		while (cost_to_home) > self.energy_budget: 
				if len(self.q) > 3:
					self.q.popleft()
					if self.command != commands.Connect:
						if self.command == commands.MoveToWaypoint:
							self.cost = commands.MoveToWaypoint.distance_finder(self.command)/self.speed
						elif self.command == commands.MoveAndCollectData:
							self.cost = commands.MoveAndCollectData.distance_finder(self.command)/self.speed
					else:
						self.cost = commands.Connect.distance_finder(self.command)/self.speed + self.sm/self.sh #estimate
					cost_to_home = self.cost + self.mission_alt/self.speed + return_dist/self.speed
				else:
					break
	def update(self): #TODO: update this for connect command instead of collect data
		if isinstance(self.command, commands.Connect):
			self.cost = self.command.cost/self.speed #TODO: make connect have cost with energy budget, go through and comment energy budget stuff so that I understand
			# Check if we are done collecting data
			if self.command.is_done():
				print(self.command.time)
				self.pop_loop()
				self.energy_budget -= self.cost
				self.energy_budget -= self.command.time * (self.sm/self.sh)
				print(self.energy_budget)
				# Finished, check is collection was successful
				if self.command.collection_success():
					print("Total collected data: " + str(self.data.data_collected))
				else:
					print("Total collected data: " + str(self.data.data_collected))
					# print("Failed collected data from " + str(self.command.node_ID))
					# Add this node to missed nodes queue
					self.missed_q.append(self.command.node_ID)
					# Check if we are done collecting data at the hovering location
				if not isinstance(self.q[0], commands.CollectData):
					if self.algorithm == "ONLINE":
						hpp_points = []
						hpp_points_id = []
						while self.missed_q:
							n = self.missed_q.pop()
							hpp_points_id.append(n)
							file1 = open(self.node_path,"r+")
							for aline in file1:
								values = aline.split()
								if int(values[0]) == n:
									east = float(values[3])
									north = float(values[4])
							file1.close()
							# Add moving-collecting from this node to the command queue
							hpp_points.append([east,north])
						if isinstance(self.q[0], commands.MoveToWaypoint):
							hpp_points.append([self.q[0].east, self.q[0].north])
							hpp_points_id.append(-1)
							hpp_points.append(commands.get_xy(vehicle))
							hpp_points_id.append(-1)
							
						else:
							hpp_points.append([self.q[0].east, self.q[0].north])
							hpp_points_id.append(-1)

						print(hpp_points)

						if len(hpp_points) < 3:
							for p, i in zip(hpp_points, hpp_points_id):
								if i == -1:
									self.q.appendleft(commands.MoveToWaypoint(p[0], p[1], self.mission_alt, vehicle))
								else:
									self.q.appendleft(commands.MoveAndCollectData(i, self.mission_alt, self.nPowers[i], vehicle, self.data, node_path = self.node_path, sim = running_sim))
						else:
							lkh = LKH_Solver([hpp_points, len(hpp_points) - 2, len(hpp_points) - 1, holder])
							LKH_path = lkh.solve()
							for p in LKH_path:
								if hpp_points_id[p] == -1:
									self.q.appendleft(commands.MoveToWaypoint(hpp_points[p][0], hpp_points[p][1], self.mission_alt, vehicle))
								else:
									self.q.appendleft(commands.MoveAndCollectData(hpp_points_id[p], self.mission_alt, self.nPowers[p], vehicle, self.data, node_path = self.node_path, sim = running_sim))

					# Done collecting data at this point, start recovery
					while self.missed_q:
						n = self.missed_q.pop()
						if self.algorithm != "NO_SUB":
							print("Added move-collect command")
								# Add moving-collecting from this node to the command queue
							self.q.appendleft(commands.MoveAndCollectData(n, self.mission_alt, self.nPowers[n], vehicle, self.data, algorithm = "NAIVE" if self.algorithm == "NAIVE" else "Default", node_path = self.node_path, sim = running_sim))
							
		# Check to see if we just finished a move-collect command
		
		if isinstance(self.command, commands.MoveAndCollectData):
			self.cost = self.command.cost/self.speed
			# Check if we are done collecting data
			if self.command.is_done():
				self.pop_loop()
				self.energy_budget -= self.cost
				self.energy_budget -= self.command.time * (self.sm/self.sh)

		
			# Check if this was the last move-collect command
			if not isinstance(self.q[0], commands.MoveAndCollectData):
				# TODO: Update the next waypoint
				pass

		if isinstance(self.command, commands.MoveToWaypoint):
			self.cost = self.command.cost/self.speed
			# Check if we are done collecting data
			if self.command.is_done():
				self.pop_loop()
				self.energy_budget -= self.cost

		if isinstance(self.command, commands.ReturnHome):
			self.cost = self.command.cost/self.speed
			self.energy_budget -= self.cost

		if isinstance(self.command, commands.Land):
			self.cost = self.mission_alt/self.speed
			self.energy_budget -= self.cost


		
		if isinstance(self.command, commands.StartTimer):
			self.data.start_timer()
			print("Starting Timer")

		if isinstance(self.command, commands.StopTimer):
			self.data.stop_timer()
			print("Stopping Timer")
			e = energy_budget.EnergyBudget(mission = self)
			self.energy_budget -= self.mission_alt/self.speed
			print(e.percentBudget())
			if self.output_path[-1] != "/":
				self.output_path += "/"

			with open(self.output_path + "flight-time.dat", "a") as tfile:
				tfile.write("0 " + str(self.data.total_time) + "\n")

			with open(self.output_path + "data_collected.dat", "a") as dfile:
				dfile.write("0 " + str(self.data.data_collected) + "\n")

			print("Data Collected:", self.data.data_collected)
			self.terminate = True

		super(CollectWSNData, self).update()



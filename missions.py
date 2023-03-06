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
from process_handler import ProcessHandler
import signal

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

def setSeed(vseed):
	global seed
	seed = vseed

def setPath(vpath):
	global path
	path = vpath
	commands.setPath(path)


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
			if vehicle.mode == VehicleMode('LAND'):
				self.dispose()
			else:
				self.update()

	# Periodically called to check command status/is-done
	def update(self):
		# Verify we haven't gone into land mode
		#TODO: More robust failsafe
		if vehicle.mode == VehicleMode('LAND'):
			self.dispose()
		# Check current command
		elif self.command.is_done():
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
		self.command_idle = commands.Idle('LOITER')
		self.command_idle.can_idle = False
		self.command = commands.GainAlt(2)
		self.q.append(self.command_idle)

	def update(self):
		if vehicle.channels['3'] > 1500 and not self.is_loiter:
			self.is_loiter = True
			self.command_idle.can_idle = True
		super(Manual, self).update()



#TODO: Refactor all CollectWSNData into single mission with an algorithm parameter
class CollectWSNData(Mission):
	name = "WSN_DATA"
	missed_q = deque()

	CMD_WAYPOINT = 0
	CMD_COLL_DATA = 1
	CMD_MSN_ALT = 2
	start_time = 0
	end_time = 0

	def __init__(self):
		commands.init_data_collected()
		# Set random seed for consistancy
		np.random.seed(seed)
		self.mission_alt = 50
		# Add take-off command
		self.q.append(commands.GainAlt(self.mission_alt))
		# Add Timer-Start command
		self.q.append(commands.StartTimer())
		# Get list of commands from file
		file1 = open(path + "drone_0_0.pln","r+")
		# Node power-settings
		self.nPowers = {-1: 0}
		# Run through each command in list
		for aline in file1:
			values = aline.split()
			# If cmd = 0 (waypoint)
			if int(values[0]) == self.CMD_WAYPOINT:
				# Add waypoint movement to queue
				self.q.append(commands.WaypointDist(float(values[1]), float(values[2]), float(values[3])))
			# else if cmd = 1 (data collection)
			elif int(values[0]) == self.CMD_COLL_DATA:
				print(values)
				# TODO: Add a normal-distribution for the nodes range
				arr = np.random.normal(float(values[2]) - 1, 8, 1)
				self.nPowers[int(values[1])] = arr[0]
				# Add collect data command
				self.q.append(commands.CollectData(int(values[1]), arr[0]))
			elif int(values[0]) == self.CMD_MSN_ALT:
				# Set mission altitude
				self.mission_alt = float(values[1])
			else:
				print("ERROR: unexpected cmd in pln file")
		file1.close()
		# Add Return-To-Home command
		self.q.append(commands.ReturnHome(self.mission_alt))
		# Add Timer-Stop command
		self.q.append(commands.StopTimer())
		if running_sim:
			# Add land command
			self.q.append(commands.Land())
		# Add first command as current command
		self.command = self.q.popleft()
		print("Node power settings")
		print(self.nPowers)

	def update(self):
		if isinstance(self.command, commands.CollectData):
			# Check if we are done collecting data
			if self.command.is_done():
				# Finished, check is collection was successful
				if self.command.collection_success():
					print("Total collected data: " + str(commands.data_collected))
				else:
					print("Total collected data: " + str(commands.data_collected))
					# print("Failed collected data from " + str(self.command.node_ID))
					# Add this node to missed nodes queue
					self.missed_q.append(self.command.node_ID)
					# Check if we are done collecting data at the hovering location
				if not isinstance(self.q[0], commands.CollectData):
					# Done collecting data at this point, start recovery
					while self.missed_q:
						print("Added move-collect command")
						n = self.missed_q.pop()
						# Add moving-collecting from this node to the command queue
						self.q.appendleft(commands.MoveAndCollectData(n, self.mission_alt, self.nPowers[n]))
		# Check to see if we just finished a move-collect command
		if isinstance(self.command, commands.MoveAndCollectData):
			# Check if this was the last move-collect command
			if not isinstance(self.q[0], commands.MoveAndCollectData):
				# TODO: Update the next waypoint
				pass
		if isinstance(self.command, commands.StartTimer):
			self.start_time = time.time()
			print("Starting Timer")
		if isinstance(self.command, commands.StopTimer):
			self.end_time = time.time()
			print("Stopping Timer")
			lapsed = self.end_time - self.start_time
			print(lapsed)
			f = open(path + "flight-time.dat", "a")
			f.write("0 " + str(lapsed) + "\n")
			f.close()
			with open(path + "data_collected.dat", "a") as dfile:
				dfile.write("0 " + str(commands.data_collected) + "\n")
			print("Data Collected:", commands.data_collected)
			commands.data_collected = 0

		super(CollectWSNData, self).update()

#TODO: Refactor all CollectWSNData into single mission with an algorithm parameter
class CollectWSNDataNaive(Mission):
	name = "WSN_DATA_NAIVE"
	missed_q = deque()

	CMD_WAYPOINT = 0
	CMD_COLL_DATA = 1
	CMD_MSN_ALT = 2
	start_time = 0
	end_time = 0

	def __init__(self):
		commands.init_data_collected()
		# Set random seed for consistancy
		np.random.seed(seed)
		self.mission_alt = 50
		# Add take-off command
		self.q.append(commands.GainAlt(self.mission_alt))
		# Add Timer-Start command
		self.q.append(commands.StartTimer())
		# Get list of commands from file
		file1 = open(path + "drone_0_0.pln","r+")
		# Node power-settings
		self.nPowers = {-1: 0}
		# Run through each command in list
		for aline in file1:
			values = aline.split()
			# If cmd = 0 (waypoint)
			if int(values[0]) == self.CMD_WAYPOINT:
				# Add waypoint movement to queue
				self.q.append(commands.WaypointDist(float(values[1]), float(values[2]), float(values[3])))
			# else if cmd = 1 (data collection)
			elif int(values[0]) == self.CMD_COLL_DATA:
				print(values)
				# TODO: Add a normal-distribution for the nodes range
				arr = np.random.normal(float(values[2]) - 1, 8, 1)
				self.nPowers[int(values[1])] = arr[0]
				# Add collect data command
				self.q.append(commands.CollectData(int(values[1]), arr[0]))
			elif int(values[0]) == self.CMD_MSN_ALT:
				# Set mission altitude
				self.mission_alt = float(values[1])
			else:
				print("ERROR: unexpected cmd in pln file")
		file1.close()
		# Add Return-To-Home command
		self.q.append(commands.ReturnHome(self.mission_alt))
		# Add Timer-Stop command
		self.q.append(commands.StopTimer())
		if running_sim:
			# Add land command
			self.q.append(commands.Land())
		# Add first command as current command
		self.command = self.q.popleft()
		print("Node power settings")
		print(self.nPowers)

	def update(self):
		if isinstance(self.command, commands.CollectData):
			# Check if we are done collecting data
			if self.command.is_done():
				# Finished, check is collection was successful
				if self.command.collection_success():
					print("Total collected data: " + str(commands.data_collected))
				else:
					print("Total collected data: " + str(commands.data_collected))
					# print("Failed collected data from " + str(self.command.node_ID))
					# Add this node to missed nodes queue
					self.missed_q.append(self.command.node_ID)
					# Check if we are done collecting data at the hovering location
				if not isinstance(self.q[0], commands.CollectData):
					# Done collecting data at this point, start recovery
					while self.missed_q:
						print("Added move-collect command")
						n = self.missed_q.pop()
						# Add moving-collecting from this node to the command queue
						self.q.appendleft(commands.MoveAndCollectDataNaive(n, self.mission_alt, self.nPowers[n]))
		# Check to see if we just finished a move-collect command
		if isinstance(self.command, commands.MoveAndCollectDataNaive):
			# Check if this was the last move-collect command
			if not isinstance(self.q[0], commands.MoveAndCollectDataNaive):
				# TODO: Update the next waypoint
				pass
		if isinstance(self.command, commands.StartTimer):
			self.start_time = time.time()
			print("Starting Timer")
		if isinstance(self.command, commands.StopTimer):
			self.end_time = time.time()
			print("Stopping Timer")
			lapsed = self.end_time - self.start_time
			print(lapsed)
			f = open(path + "flight-time.dat", "a")
			f.write("1 " + str(lapsed) + "\n")
			f.close()
			with open(path + "data_collected.dat", "a") as dfile:
				dfile.write("1 " + str(commands.data_collected) + "\n")
			print("Data Collected:", commands.data_collected)
			commands.data_collected = 0

		super(CollectWSNDataNaive, self).update()

#TODO: Refactor all CollectWSNData into single mission with an algorithm parameter - This one is still buggy and fails on execution. Either fix or remove
class CollectWSNDataLKH(Mission):
	name = "WSN_DATA_LKH"
	missed_q = deque()

	CMD_WAYPOINT = 0
	CMD_COLL_DATA = 1
	CMD_MSN_ALT = 2
	start_time = 0
	end_time = 0

	def __init__(self):
		commands.init_data_collected()
		# Set random seed for consistancy
		np.random.seed(seed)
		self.mission_alt = 50
		# Add take-off command
		self.q.append(commands.GainAlt(self.mission_alt))
		# Add Timer-Start command
		self.q.append(commands.StartTimer())
		# Get list of commands from file
		file1 = open(path + "drone_0_0.pln","r+")
		# Node power-settings
		self.nPowers = {-1: 0}
		# Run through each command in list
		for aline in file1:
			values = aline.split()
			# If cmd = 0 (waypoint)
			if int(values[0]) == self.CMD_WAYPOINT:
				# Add waypoint movement to queue
				self.q.append(commands.WaypointDist(float(values[1]), float(values[2]), float(values[3])))
			# else if cmd = 1 (data collection)
			elif int(values[0]) == self.CMD_COLL_DATA:
				print(values)
				# TODO: Add a normal-distribution for the nodes range
				arr = np.random.normal(float(values[2]) - 1, 8, 1)
				self.nPowers[int(values[1])] = arr[0]
				# Add collect data command
				self.q.append(commands.CollectData(int(values[1]), arr[0]))
			elif int(values[0]) == self.CMD_MSN_ALT:
				# Set mission altitude
				self.mission_alt = float(values[1])
			else:
				print("ERROR: unexpected cmd in pln file")
		file1.close()
		# Add Return-To-Home command
		self.q.append(commands.ReturnHome(self.mission_alt))
		# Add Timer-Stop command
		self.q.append(commands.StopTimer())
		if running_sim:
			# Add land command
			self.q.append(commands.Land())
		# Add first command as current command
		self.command = self.q.popleft()
		print("Node power settings")
		print(self.nPowers)

	def update(self):
		if isinstance(self.command, commands.CollectData):
			# Check if we are done collecting data
			if self.command.is_done():
				# Finished, check is collection was successful
				if self.command.collection_success():
					print("Total collected data: " + str(commands.data_collected))
				else:
					print("Total collected data: " + str(commands.data_collected))
					# print("Failed collected data from " + str(self.command.node_ID))
					# Add this node to missed nodes queue
					self.missed_q.append(self.command.node_ID)
					# Check if we are done collecting data at the hovering location
				if not isinstance(self.q[0], commands.CollectData):
					# Done collecting data at this point, start recovery
					hpp_points = []
					hpp_points_id = []
					while self.missed_q:
						print("Added move-collect command")
						n = self.missed_q.pop()
						hpp_points_id.append(n)
						file1 = open(path + "node_info.txt","r+")
						for aline in file1:
							values = aline.split()
							if int(values[0]) == n:
								east = float(values[3])
								north = float(values[4])
						file1.close()
						# Add moving-collecting from this node to the command queue
						hpp_points.append([east,north])
					if isinstance(self.q[0], commands.WaypointDist):
						hpp_points.append(commands.get_xy())
						hpp_points.append(self.q[0].east, self.q[0].north)
					else:
						hpp_points.append(commands.get_xy())
						hpp_points.append(commands.get_xy())
					PrintLKHFile(hpp_points, len(hpp_points) - 2, len(hpp_points) - 1)

					#TODO Solve HPP using LKH here.
					path = getLKHSolution(len(hpp_points) - 2, len(hpp_points) - 1)
					for p in path:
						self.q.appendleft(commands.MoveAndCollectData(hpp_points_id[p], self.mission_alt, self.nPowers[n]))
		# Check to see if we just finished a move-collect command
		if isinstance(self.command, commands.MoveAndCollectData):
			# Check if this was the last move-collect command
			if not isinstance(self.q[0], commands.MoveAndCollectData):
				# TODO: Update the next waypoint
				pass
		if isinstance(self.command, commands.StartTimer):
			self.start_time = time.time()
			print("Starting Timer")
		if isinstance(self.command, commands.StopTimer):
			self.end_time = time.time()
			print("Stopping Timer")
			lapsed = self.end_time - self.start_time
			print(lapsed)
			f = open(path + "flight-time.dat", "a")
			f.write("2 " + str(lapsed) + "\n")
			f.close()
			with open(path + "data_collected.dat", "a") as dfile:
				dfile.write("2 " + str(commands.data_collected) + "\n")
			print("Data Collected:", commands.data_collected)
			commands.data_collected = 0

		super(CollectWSNDataLKH, self).update()

#Prints LKH parameters to a the required file
def PrintLKHFile(hpp_points, start, end):
	with open("FixedHPP.par", "w") as parFile:
		parFile.write("PROBLEM_FILE = FixedHPP.tsp\n")
		parFile.write("COMMENT Fixed Hamiltonian Path Problem\n")
		parFile.write("TOUR_FILE = LKH_output.dat\n")

	with open("FixedHPP.tsp", "w") as dataFile:
		dataFile.write("NAME : FixedHPP \n")
		dataFile.write("COMMENT : Fixed Hamiltonian Path Problem \n")
		dataFile.write("TYPE : TSP \n")
		dataFile.write("DIMENSION : " + set(len(hpp_points)) + "\n")
		dataFile.write("EDGE_WEIGHT_TYPE : EXPLICIT \n")
		dataFile.write("EDGE_WEIGHT_FORMAT : FULL_MATRIX\n")

		dataFile.write("EDGE_WEIGHT_SECTION\n")
		for i in range(len(hpp_points)):
			for j in range(len(hpp_points)):
				if(i == start and j == end) or (i == end and j == start):
					dataFile.write("0.0\t")
				else:
					if i == start or i == end or j == start or j == end:
						dataFile.write(str(FindNodeDistance(hpp_points[i], hpp_points[j])*1000) + "\t")
					else:
						dataFile.write(str(FindNodeDistance(hpp_points[i], hpp_points[j])) + "\t")
			dataFile.write("\n")
		dataFile.write("EOF\n")

#Finds the distance between two points
def FindNodeDistance(a, b):
	return math.sqrt(((a[0] - b[0])**2 + (a[1] - b[1])**2))

#Runs the LKH solver and returns the solution
def getLKHSolution(start, end):
	lkh_process = sb.Popen([defines.LKH_PATH, "FixedHPP.par"])

	# process_num = ph.add_process(lkh_process)
	lkh_process.communicate()[0]
	# ph.remove_process(process_num)

	with open("LKH_output.dat") as outputFile:
		lines = outputFile.readlines()
	path = []

	for i in range(6, len(lines)):
		path.append(int(lines[i]))

	if path[0] != start or path[-1] != end:
		#Something is wrong
		if (path[0] == start and path[-1] != end) or (path[0] != start and path[-1] == end):
			raise(Exception("Temp exception 1 for TSP failing to function as HPP"))
		
		while path[0] != start:
			print("Rotating list...")
			path.append(path[0])
			path.pop(0)

		if path[1] == end:
			path.append(path[0])
			path.pop(0)
			path.reverse()

		if path[-1] != end:
			raise(Exception("Temp exception 2 for TSP failing to function as HPP"))
	

	return path
	
#TODO: Refactor all CollectWSNData into single mission with an algorithm parameter
class CollectWSNDataNoSub(Mission):
	name = "WSN_DATA_NO_SUB"
	missed_q = deque()

	CMD_WAYPOINT = 0
	CMD_COLL_DATA = 1
	CMD_MSN_ALT = 2
	start_time = 0
	end_time = 0

	def __init__(self):
		commands.init_data_collected()
		# Set random seed for consistancy
		np.random.seed(seed)
		self.mission_alt = 50
		# Add take-off command
		self.q.append(commands.GainAlt(self.mission_alt))
		# Add Timer-Start command
		self.q.append(commands.StartTimer())
		# Get list of commands from file
		file1 = open(path + "drone_0_0.pln","r+")
		# Node power-settings
		self.nPowers = {-1: 0}
		# Run through each command in list
		for aline in file1:
			values = aline.split()
			# If cmd = 0 (waypoint)
			if int(values[0]) == self.CMD_WAYPOINT:
				# Add waypoint movement to queue
				self.q.append(commands.WaypointDist(float(values[1]), float(values[2]), float(values[3])))
			# else if cmd = 1 (data collection)
			elif int(values[0]) == self.CMD_COLL_DATA:
				print(values)
				# TODO: Add a normal-distribution for the nodes range
				arr = np.random.normal(float(values[2]) - 1, 8, 1)
				self.nPowers[int(values[1])] = arr[0]
				# Add collect data command
				self.q.append(commands.CollectData(int(values[1]), arr[0]))
			elif int(values[0]) == self.CMD_MSN_ALT:
				# Set mission altitude
				self.mission_alt = float(values[1])
			else:
				print("ERROR: unexpected cmd in pln file")
		file1.close()
		# Add Return-To-Home command
		self.q.append(commands.ReturnHome(self.mission_alt))
		# Add Timer-Stop command
		self.q.append(commands.StopTimer())
		if running_sim:
			# Add land command
			self.q.append(commands.Land())
		# Add first command as current command
		self.command = self.q.popleft()
		print("Node power settings")
		print(self.nPowers)

	def update(self):
		if isinstance(self.command, commands.CollectData):
			# Check if we are done collecting data
			if self.command.is_done():
				# Finished, check is collection was successful
				if self.command.collection_success():
					print("Total collected data: " + str(commands.data_collected))
				else:
					print("Total collected data: " + str(commands.data_collected))
					# Add this node to missed nodes queue
					# self.missed_q.append(self.command.node_ID)
					# Check if we are done collecting data at the hovering location
				if not isinstance(self.q[0], commands.CollectData):
					# Done collecting data at this point, start recovery
					# while self.missed_q:
						# print("Added move-collect command")
						# n = self.missed_q.pop()
						# Add moving-collecting from this node to the command queue
						#self.q.appendleft(commands.MoveAndCollectData(n, self.mission_alt, self.nPowers[n]))
					pass
		# Check to see if we just finished a move-collect command
		if isinstance(self.command, commands.MoveAndCollectData):
			# Check if this was the last move-collect command
			if not isinstance(self.q[0], commands.MoveAndCollectData):
				# TODO: Update the next waypoint
				pass
		if isinstance(self.command, commands.StartTimer):
			self.start_time = time.time()
			print("Starting Timer")
		if isinstance(self.command, commands.StopTimer):
			self.end_time = time.time()
			print("Stopping Timer")
			lapsed = self.end_time - self.start_time
			print(lapsed)
			f = open(path + "flight-time.dat", "a")
			f.write("3 " + str(lapsed) + "\n")
			f.close()
			
			with open(path + "data_collected.dat", "a") as dfile:
				dfile.write("3 " + str(commands.data_collected) + "\n")
			print("Data Collected:", commands.data_collected)
			commands.data_collected = 0

		super(CollectWSNDataNoSub, self).update()
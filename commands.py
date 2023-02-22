import abc
from dronekit import VehicleMode
from pymavlink import mavutil
import math
import time
import subprocess as sp
from threading import Thread, Event
import defines




def init_data_collected():
	global data_collected
	data_collected = 0

def setSimulation(sim):
	global running_sim
	running_sim = sim

def pass_vehicle(passed_vehicle):
	global vehicle
	vehicle = passed_vehicle

def setPath(mpath):
	global path
	path = mpath

def get_xy():
	return [vehicle.location.local_frame.east, vehicle.location.local_frame.north]


class Command(object):
	__metaclass__ = abc.ABCMeta

	def __init(self):
		pass

	@abc.abstractmethod
	def init(self):
		pass

	def update(self):
		time.sleep(0.1)

	@abc.abstractmethod
	def is_done(self):
		pass


class GainAlt(Command):
	def __init__(self, target_altitude):
		self.target_altitude = target_altitude

	def init(self):
		vehicle.mode = VehicleMode('GUIDED')

		if vehicle.location.global_relative_frame.alt < 1:
			while not vehicle.armed:
				vehicle.armed = True
				time.sleep(0.5)
			vehicle.simple_takeoff(self.target_altitude)
		else:
			goto_position_target_local_enu(0, 0, self.target_altitude)

	def is_done(self):
		diff = abs(vehicle.location.global_relative_frame.alt - self.target_altitude)
		return diff < 0.5

class StartTimer(Command):
	def __init__(self):
		global data_collected
		data_collected = 0
		pass

	def init(self):
		pass

	def is_done(self):
		return True

class StopTimer(Command):
	def __init__(self):
		pass

	def init(self):
		pass

	def is_done(self):
		return True


class WaypointDist(Command):
	def __init__(self, east, north, up):
		self.east = east
		self.north = north
		self.up = up

	def init(self):
		vehicle.mode = VehicleMode('GUIDED')
		goto_position_target_local_enu(self.east, self.north, self.up)

	def is_done(self):
		target_dist = abs(math.sqrt(
			(vehicle.location.local_frame.north - self.north) ** 2 + 
			(vehicle.location.local_frame.east - self.east) ** 2 + 
			(vehicle.location.local_frame.down + self.up) ** 2))
		return target_dist < 0.5


class WaypointTime(Command):
	def __init__(self, east, north, up, seconds):
		self.east = east
		self.north = north
		self.up = up
		self.seconds = seconds

	def init(self):
		vehicle.mode = VehicleMode('GUIDED')
		goto_position_target_local_enu(self.east, self.north, self.up)
		self.time_start = time.time()

	def is_done(self):
		seconds_elapsed = time.time() - self.time_start 
		return seconds_elapsed > self.seconds


class Idle(Command):
	def __init__(self, mode):
		self.mode = mode
		self.can_idle = True
		self.is_idle = False

	def init(self):
		vehicle.mode = VehicleMode('GUIDED')

	def update(self):
		if self.can_idle and not self.is_idle:
			vehicle.mode = VehicleMode(self.mode)
			self.is_idle = True
		time.sleep(0.1)

	def is_done(self):
		return False


class ReturnHome(Command):
	def __init__(self, alt):
		self.east = 0
		self.north = 0
		self.up = alt

	def init(self):
		vehicle.mode = VehicleMode('GUIDED')
		goto_position_target_local_enu(self.east, self.north, self.up)

	def is_done(self):
		target_dist = abs(math.sqrt(
			(vehicle.location.local_frame.north - self.north) ** 2 + 
			(vehicle.location.local_frame.east - self.east) ** 2 + 
			(vehicle.location.local_frame.down + self.up) ** 2))
		return target_dist < 0.5


class Land(Command):
	def __init__(self):
		pass

	def init(self):
		land_in_place()

	def is_done(self):
		return vehicle.location.global_relative_frame.alt < 0.1


class CollectData(Command):
	# Collect data from node, with node communication range node_range (for simulation)
	def __init__(self, node, power):
		self.node_ID = node
		self.power = power
		self.east = 0
		self.north = 0
		self.thread = None
		self.node_hostname = None
		self.node_collect_time = None
		# Find data about this node
		file1 = open( path + "node_info.txt","r+")
		for aline in file1:
			values = aline.split()
			if int(values[0]) == self.node_ID:
				self.node_hostname = values[1]
				self.node_collect_time = int(values[2])
				self.east = float(values[3])
				self.north = float(values[4])
				break
		file1.close()
		# Thread events for collection success/complete
		self.collect_success = Event()
		self.collect_success.clear()
		self.collect_complete = Event()
		self.collect_complete.clear()

	def init(self):
		# Create thread for comms process
		if self.node_collect_time is not None:
			self.thread = Thread(target=self.launchCollection)
			self.thread.start()
		else:
			print("ERROR: failed to find node " + str(self.node_ID) + " info")
			# Set complete-flag
			self.collect_complete.set()

	def launchCollection(self):
		# Start comms process, wait for response
		#print("Starting data collection process")

		# Are we running the simulation?
		if running_sim:
			dist_to_node = abs(math.sqrt(
				(vehicle.location.local_frame.north - self.north) ** 2 + 
				(vehicle.location.local_frame.east - self.east) ** 2 + 
				(vehicle.location.local_frame.down) ** 2))
			# Collect data using NS3
			child = sp.Popen([defines.NS_3_PATH, "run", "scratch/drone-to-sensor", "--", "--distance="+str(dist_to_node), "--payload=5000000", "--txpower="+str(self.power), "--delay=true"],  stdout=sp.DEVNULL)
			child.communicate()[0]
			rc = child.returncode
		else:
			# Collect data using collect_data executable
			child = sp.Popen(["/home/pi/MinLatencyWSN/MinLat_autopilot/Networking/Client/collect_data", str(self.node_ID), str(self.node_hostname), str(self.node_collect_time)], stdout=sp.DEVNULL)
			child.communicate()[0]
			rc = child.returncode

		# If return on comms process was successful, set success-flag
		if rc == 0:
			global data_collected
			if data_collected is None:
				data_collected = 0
			print("Successfully collected 5000000 from node " + str(self.node_ID))
			data_collected += 5000000
			self.collect_success.set()
		# else, leave success-flag unset
		else:
			print("failed to collect from node " + str(self.node_ID))
		
		# Set complete-flag, rejoin
		self.collect_complete.set()

	def is_done(self):
		if self.collect_complete.is_set():
			if self.thread is not None:
				self.thread.join()
			return True
		else:
			return False
		
	def collection_success(self):
		return self.collect_success.is_set()


class MoveAndCollectData(Command):
	# Attempt to collect data from node, while moving towards the node 
	# at altitude alt, with node communication range node_range (for simulation)
	def __init__(self, node, alt, power):
		self.node_ID = node
		self.power = power
		self.east = 0
		self.north = 0
		self.up = alt
		self.node_hostname = None
		self.node_collect_time = None
		# Find data about this node
		file1 = open(path + "node_info.txt","r+")
		for aline in file1:
			values = aline.split()
			if int(values[0]) == self.node_ID:
				self.node_hostname = values[1]
				self.node_collect_time = int(values[2])
				self.east = float(values[3])
				self.north = float(values[4])
				break
		file1.close()
		self.collect_success = False
		# Thread event for collection complete
		self.collect_complete = Event()
		self.collect_complete.clear()
		self.thread = None

	def init(self):
		print("Starting move-collect command for node ", self.node_ID)
		# Move towards the node
		vehicle.mode = VehicleMode('GUIDED')
		goto_position_target_local_enu(self.east, self.north, self.up)

	def update(self):
		# Check if we found data on this node
		if self.node_collect_time is not None:
			# Attempt to communicate with this node
			if not self.collect_success and self.launchCollection():
				print("Connected to node, stopping for data collection")
				# Made contact, stop and collect data
				send_stop()
				self.collect_success = True
				self.thread = Thread(target=self.collection_thread)
				self.thread.start()
			else:
				# Still cannot talk to node...
				target_dist = abs(math.sqrt(
					(vehicle.location.local_frame.north - self.north) ** 2 + 
					(vehicle.location.local_frame.east - self.east) ** 2 + 
					(vehicle.location.local_frame.down + self.up) ** 2))
				#  Check if we reached the node
				if target_dist < 0.5:
					# Reached node, could not connect...
					self.collect_complete.set()
		else:
			print("ERROR: failed to find node " + str(self.node_ID) + " info")
			# Set complete-flag
			self.collect_complete.set()

	def launchCollection(self):
		# Start comms process with 0 wait time, wait for response
		
		# Are we running the simulation?
		if running_sim:
			dist_to_node = abs(math.sqrt(
				(vehicle.location.local_frame.north - self.north) ** 2 + 
				(vehicle.location.local_frame.east - self.east) ** 2 + 
				(vehicle.location.local_frame.down) ** 2))
			# Attempt to contact node using NS3, disable delay, set data to 1 byte
			child = sp.Popen([defines.NS_3_PATH, "run", "scratch/drone-to-sensor", "--", "--distance="+str(dist_to_node), "--payload=5000000", "--txpower="+str(self.power), "--delay=false"], stdout=sp.DEVNULL)
			child.communicate()[0]
			rc = child.returncode
		else:
			# Attempt to contact node using collect_data executable with transmission time = 0
			child = sp.Popen(["/home/pi/MinLatencyWSN/MinLat_autopilot/Networking/Client/collect_data", str(self.node_ID), str(self.node_hostname), "0"], stdout=sp.DEVNULL)
			child.communicate()[0]
			rc = child.returncode

		# If return on comms process was successful, return true
		if rc == 0:
			print("Successfully collected 5000000 from node " + str(self.node_ID))
			global data_collected
			data_collected += 5000000
			return True
		# else, return false
		else:
			print("failed to collect from node " + str(self.node_ID))
			return False

	def collection_thread(self):
		time.sleep(self.node_collect_time/1000.0)
		self.collect_complete.set()

	def is_done(self):
		if self.collect_complete.is_set():
			return True
		else:
			return False
		
	def collection_success(self):
		return self.collect_success

class MoveAndCollectDataNaive(Command):
	# Attempt to collect data from node, while moving towards the node 
	# at altitude alt, with node communication range node_range (for simulation)
	def __init__(self, node, alt, power):
		self.node_ID = node
		self.power = power
		self.east = 0
		self.north = 0
		self.up = alt
		self.node_hostname = None
		self.node_collect_time = None
		# Find data about this node
		file1 = open(path + "node_info.txt","r+")
		for aline in file1:
			values = aline.split()
			if int(values[0]) == self.node_ID:
				self.node_hostname = values[1]
				self.node_collect_time = int(values[2])
				self.east = float(values[3])
				self.north = float(values[4])
				break
		file1.close()
		self.collect_success = False
		self.stopped_at_point = False
		# Thread event for collection complete
		self.collect_complete = Event()
		self.collect_complete.clear()
		self.thread = None

	def init(self):
		print("Starting move-collect command for node ", self.node_ID)
		# Move towards the node
		vehicle.mode = VehicleMode('GUIDED')
		goto_position_target_local_enu(self.east, self.north, self.up)

	def update(self):
		# Check if we found data on this node
		if self.node_collect_time is not None:

			# Find Distance to node
			target_dist = abs(math.sqrt(
					(vehicle.location.local_frame.north - self.north) ** 2 + 
					(vehicle.location.local_frame.east - self.east) ** 2 + 
					(vehicle.location.local_frame.down + self.up) ** 2))
			# Attempt to communicate with this node
			if not self.collect_success and self.launchCollection():
				print("Connected to node, starting data collection")
				# Made contact, collect data
				self.collect_success = True
				self.thread = Thread(target=self.collection_thread)
				self.thread.start()
			elif self.collect_success and target_dist < 0.5 and not self.stopped_at_point:
				send_stop()
				print("Stopping at node for data collection")
				self.stopped_at_point = True
			else:
				# Still cannot talk to node...
				#  Check if we reached the node
				if target_dist < 0.5 and not self.collect_success:
					# Reached node, could not connect...
					self.collect_complete.set()
			#Stop if collecting and have reached the waypoint
			
		else:
			print("ERROR: failed to find node " + str(self.node_ID) + " info")
			# Set complete-flag
			self.collect_complete.set()

	def launchCollection(self):
		# Start comms process with 0 wait time, wait for response
		
		# Are we running the simulation?
		if running_sim:
			dist_to_node = abs(math.sqrt(
				(vehicle.location.local_frame.north - self.north) ** 2 + 
				(vehicle.location.local_frame.east - self.east) ** 2 + 
				(vehicle.location.local_frame.down) ** 2))
			# Attempt to contact node using NS3, disable delay, set data to 1 byte
			child = sp.Popen([defines.NS_3_PATH, "run", "scratch/drone-to-sensor", "--", "--distance="+str(dist_to_node), "--payload=5000000", "--txpower="+str(self.power), "--delay=false"])
			child.communicate()[0]
			rc = child.returncode
		else:
			# Attempt to contact node using collect_data executable with transmission time = 0
			child = sp.Popen(["/home/pi/MinLatencyWSN/MinLat_autopilot/Networking/Client/collect_data", str(self.node_ID), str(self.node_hostname), "0"], stdout=sp.DEVNULL)
			child.communicate()[0]
			rc = child.returncode

		# If return on comms process was successful, return true
		if rc == 0:
			global data_collected
			print("Successfully collected 5000000 from node " + str(self.node_ID))
			data_collected += 5000000
			return True
		# else, return false
		else:
			print("failed to collect from node " + str(self.node_ID))
			return False

	def collection_thread(self):
		time.sleep(self.node_collect_time/1000.0)
		self.collect_complete.set()

	def is_done(self):
		if self.collect_complete.is_set():
			return True
		else:
			return False
		
	def collection_success(self):
		return self.collect_success


def land_in_place():
	vehicle.mode = VehicleMode("LAND")
	print("Landing")

	# Wait until the vehicle reaches the ground
	while True:
		# print(" Altitude: ", vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt < 0.1:
			print("Landed")
			break
		time.sleep(1)

	vehicle.close()

def goto_position_target_local_enu(east, north, up):
	down = -up
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
			0,      # time_boot_ms (not used)
			0, 0,   # target system, target component
			mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
			0b0000111111111000, # type_mask (only positions enabled)
			north, east, down, # NED positions
			0, 0, 0, # NED velocities
			0, 0, 0, # NED accelerations (not supported),
			0, 0)    # yaw, yaw_rate (not supported)
	vehicle.send_mavlink(msg)

def send_stop():
	"""
	Move vehicle in direction based on specified velocity vectors.
	"""
	msg = vehicle.message_factory.set_position_target_global_int_encode(
		0,       # time_boot_ms (not used)
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
		0b0000111111000111, # type_mask (only speeds enabled)
		0, # lat_int - X Position in WGS84 frame in 1e7 * meters
		0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
		0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
		# altitude above terrain if GLOBAL_TERRAIN_ALT_INT
		0, # X velocity in NED frame in m/s
		0, # Y velocity in NED frame in m/s
		0, # Z velocity in NED frame in m/s
		0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
	# Send message once
	vehicle.send_mavlink(msg)

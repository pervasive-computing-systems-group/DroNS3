import time
import missions
import commands
from dronekit import connect, VehicleMode, LocationGlobalRelative
import subprocess as sb
import sys
import defines
from mission_wrapper import Wrapper
# 
# To run the simulation, open a terminal in ardupilot/ArduCopter and run:
# sim_vehicle.py -f quad -L CSM_SurveyField --console --map --osd
# 

class Custom_Command(commands.Command):
	def __init__(self, vehicle, mission, args, debug = False):
		self.vehicle = vehicle
		self.mission = mission
		self.args = args
		self.done = False
		self.debug = debug
		print(self.args)

	def begin(self):
		for i in range(0,len(self.args), 3):
			command = commands.MoveToWaypoint(self.args[i], self.args[i+1], self.args[i+2], self.vehicle, debug = self.debug)
			self.mission.q.appendleft(command)
			
		self.done = True

	def is_done(self):
		return self.done


if __name__ == '__main__':
	w = Wrapper()
	w.start_mission(mission=missions.General(w.vehicle, mission_file="sample_wsn_mission/basic_data_collect.pln", debug = True, custom_commands=[Custom_Command], is_sim=True))
	#w.start_mission(mission=missions.ConnectionTests())
	#  Mission file used in ``Holistic path planning for multi-drone data collection''
	# w.start_mission(mission=missions.CollectWSNData(plan_path="sample_wsn_mission/simple_plan.pln", node_path = "sample_wsn_mission/simple_node_info.txt", output_path = "./", algorithm = "DEFAULT"))
	

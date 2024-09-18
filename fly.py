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


if __name__ == '__main__':
	w = Wrapper(is_sim=True)
	#w.start_mission(mission=missions.General(w.vehicle, mission_file="general_mission.txt", debug = True, custom_commands=[Custom_Command], is_sim=True))
	w.start_mission(mission=missions.ConnectionTests())
	#w.start_mission(mission=missions.CollectWSNData(plan_path="sample_wsn_mission/drone_plan.pln", node_path = "sample_wsn_mission/node_info.txt", output_path = "./", algorithm = "DEFAULT"))
	

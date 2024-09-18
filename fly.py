import missions
import subprocess as sb
from mission_wrapper import Wrapper

# To run the simulation, open a terminal in ardupilot/ArduCopter and run:
# sim_vehicle.py -f quad -L CSM_SurveyField --console --map --osd


if __name__ == '__main__':
	w = Wrapper(is_sim=False)
	w.start_mission(mission=missions.General(w.vehicle, mission_file="sample_wsn_mission.flight_test.pln", debug = True, custom_commands=[Custom_Command], is_sim=True))
	# w.start_mission(mission=missions.ConnectionTests())
	#w.start_mission(mission=missions.CollectWSNData(plan_path="sample_wsn_mission/drone_plan.pln", node_path = "sample_wsn_mission/node_info.txt", output_path = "./", algorithm = "DEFAULT"))
	
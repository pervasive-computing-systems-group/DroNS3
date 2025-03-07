import missions
import subprocess as sb
from mission_wrapper import Wrapper

MISSION_FILE = "/home/pi/DroNS3/plan/field_day.pln"

# To run the simulation, open a terminal in ardupilot/ArduCopter and run:
# sim_vehicle.py -f quad -L CSM_SurveyField --console --map --osd


if __name__ == "__main__":
	w = Wrapper(is_sim=False)
	w.start_mission(
		mission=missions.WSNMission(
			w.vehicle,
			mission_file=MISSION_FILE,
			debug=True,
			is_sim=False,
		)
	)

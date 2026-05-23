#Path to NS_3 executable from the root directory (i.e. "/home/...")
NS_3_PATH = "/home/minespecs/NS3/ns-allinone-3.36.1/ns-3.36.1/ns3"

## Holistic planner paths
# Path to the orchestrator director (i.e. "/home/...")
ORCHESTRATOR_PATH = "/home/jonathan/git/HolisticFramework/"
# Path to SimpleNetSim executable from the root directory (i.e. "/home/...")
SNS_PATH = ORCHESTRATOR_PATH+"DroNS3/SimpleNetSim/Simulation/sim"
# Path to LocalPlanner from the root directory (i.e. "/home/...")
LOCAL_PLANNER_PATH = ORCHESTRATOR_PATH+"MissionPlanner/build/local-planner"
# Path of sim-output folder from the root directory (i.e. "/home/...")
SIM_OUT_PATH = ORCHESTRATOR_PATH+"sim_out/"

## Other Holistic planner slop
ENABLE_RTB = True
BATTERY_BUFFER = 0.05
TOTAL_BATTERY = 150000

# Path to the mission directory from the root directory (i.e. "/home/...")
MISSION_PATH = "/home/minespecs/RDS/sample_wsn_mission/"
# Path to the LKH executable from the root directory
LKH_PATH = "/home/jonathan/bin/LKH"
# IP Address of current machine
IP_ADDRESS = "127.0.0.1"
# Enables debug prints
debug = True
# Communications Protocol PAth
Comms_Path = None

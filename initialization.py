import global_variables
import robot_callbacks
import robot_pathing

from os import environ
import rospy
from lab_utils.ip_config import get_ip
import os, sys

VOTRE_NUMERO_EQUIPE = 11
ADMIN = False
#VOTRE_NUMERO_ROBOT = 7
VOTRE_NUMERO_ROBOT = 4  # Works well

# ROS subscribers and publishers
def initialize_subscribers():
    robot_callbacks.subscribe()

def initialize_all():
    # Set SIMULATION mode True or False
    global_variables.SIMULATION_STATUS = True

    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

    if global_variables.SIMULATION_STATUS == True:
        environ['ROS_MASTER_URI'] = f"http://localhost:113{70+VOTRE_NUMERO_EQUIPE if not ADMIN else 11}/"
        print("SIMULATION MODE")
    else:
        environ['ROS_MASTER_URI'] = "http://cpr-ets05-0{}.local:11311/".format(VOTRE_NUMERO_ROBOT)
        environ['ROS_IP'] = get_ip() # Workstation IP address
        print("REAL MODE")

    rospy.init_node('dingo_controller', anonymous=True)
    rate = rospy.Rate(50)

    # Subscribers initialization for callbacks
    initialize_subscribers()
    rospy.sleep(5)

    # Call all init functions here
    robot_pathing.robot_pathing_init()

    print("Initialization done")

    return rate


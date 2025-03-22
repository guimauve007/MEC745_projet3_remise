import robot_callbacks
import tag_detection
import robot_pathing

from os import environ
import rospy
from lab_utils.ip_config import get_ip
import os, sys

VOTRE_NUMERO_EQUIPE = 11
ADMIN = False
VOTRE_NUMERO_ROBOT = 7

SIM = True


# ROS subscribers et publishers
def initialize_subscribers():
    robot_callbacks.subscribe(SIM)

def initialize_all():
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

    if SIM:
        environ['ROS_MASTER_URI'] = f"http://localhost:113{70+VOTRE_NUMERO_EQUIPE if not ADMIN else 11}/"
    else:
        environ['ROS_MASTER_URI'] = "http://cpr-ets05-0{}.local:11311/".format(VOTRE_NUMERO_ROBOT)
        environ['ROS_IP'] = get_ip() # adresse IP de votre station de travail

    rospy.init_node('dingo_controller', anonymous=True)
    rate = rospy.Rate(50)

    # Subscribers initialization for callbacks
    initialize_subscribers()

    # Call all init functions here
    tag_detection.init_tag_detection()
    robot_pathing.robot_pathing_init()

    print("Initialization done")

    return rate


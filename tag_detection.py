import rospy
from apriltag_ros.msg import AprilTagDetectionArray
#import tkinter as tk
import numpy as np
from scipy.spatial.transform import Rotation as R
import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from lab_utils.ip_config import get_ip
from os import environ


# *************************************************************************************************

VOTRE_NUMERO_EQUIPE = 11
SIM = True
ADMIN = False
VOTRE_NUMERO_ROBOT = 7
    
if SIM:
    environ['ROS_MASTER_URI'] = f"http://localhost:113{70+VOTRE_NUMERO_EQUIPE if not ADMIN else 11}/"
else:
    environ['ROS_MASTER_URI'] = "http://cpr-ets05-0{}.local:11311/".format(VOTRE_NUMERO_ROBOT)
    environ['ROS_IP'] = get_ip() # adresse IP de votre station de travail

rospy.init_node('dingo_controller', anonymous=True)
rate = rospy.Rate(50)

# *************************************************************************************************


# Apriltag callback
tag_array = AprilTagDetectionArray()
def tag_callback(msg):
    global tag_array
    tag_array  = msg

def get_heading_from_quaternion(q):
    if(np.sqrt(q.x**2+q.y**2+q.z**2+q.w**2) == 0):
        print('Quaternion norm 0 : ', q.x, q.y, q.z, q.w)
        return 0
    else:
        r = R.from_quat([q.x, q.y, q.z, q.w])
        angles = r.as_euler('xyz', degrees=False)
        return angles[2]

def init_tag_detection():
    tag_sub = rospy.Subscriber('/mobile_manip/tag_detections', AprilTagDetectionArray, tag_callback)

#root = tk.Tk()

def updateTagDetectionData(x_value, y_value, z_value, orientation_value):
    if (tag_array.detections == []):
        x_value.set(0.00)
        y_value.set(0.00)
        z_value.set(0.00)
        orientation_value.set(0.00)
    else: # Affiche le premier tag de la liste detections
        x_value.set('{:2f}'.format(tag_array.detections[0].pose.pose.pose.position.x))
        y_value.set('{:2f}'.format(tag_array.detections[0].pose.pose.pose.position.y))
        z_value.set('{:2f}'.format(tag_array.detections[0].pose.pose.pose.position.z))
        orientation_value.set('{:2f}'.format(get_heading_from_quaternion(tag_array.detections[0].pose.pose.pose.orientation)))
        
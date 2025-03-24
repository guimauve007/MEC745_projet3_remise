import global_variables

import numpy as np
from scipy.spatial.transform import Rotation as R
from homogeneous_transform import rh, th
from jackal_msgs.msg import Drive
import robot_callbacks
import rospy
from mobile_manip.srv import ReachName, GetValues, ReachValues
import tkinter as tk
import os, sys
current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from lab_utils.ip_config import get_ip
from os import environ

# paramètres du robot
INTERWHEEL_DISTANCE = 0.434 # m
LEFT_WHEEL_RADIUS = 0.049 # m
RIGHT_WHEEL_RADIUS = 0.049 # m

# Vitesses pour le déplacement manuel
MANUAL_LINEAR_SPEED = 0.5 # m/s
MANUAL_ANGULAR_SPEED = 0.5 # rad/s

def move_robot(linear_speed, angular_speed):
    vel_left  = (linear_speed - angular_speed * INTERWHEEL_DISTANCE / 2.0) / LEFT_WHEEL_RADIUS
    vel_right = (linear_speed + angular_speed * INTERWHEEL_DISTANCE / 2.0) / RIGHT_WHEEL_RADIUS

    # Envoi des commandes au roues par topic ROS
    cmd_drive_msg = Drive()
    cmd_drive_msg.drivers[0] = vel_left
    cmd_drive_msg.drivers[1] = vel_right
    robot_callbacks.publish_cmd_drive(cmd_drive_msg)

def turn_robot_lef():
    move_robot(0, get_manual_angular_speed())

def turn_robot_right():
    move_robot(0, -get_manual_angular_speed())

def stop_robot():
    move_robot(0,0)

def get_manual_linear_speed():
    return MANUAL_LINEAR_SPEED

def get_manual_angular_speed():
    return MANUAL_LINEAR_SPEED

def arm_reach_coord(x, y, z):
    effector_goal = [x, y, z, 0, 90, 0, 0]
    print(effector_goal)
    rospy.wait_for_service('/mobile_manip/reach_cartesian')
    reach_pose = rospy.ServiceProxy('/mobile_manip/reach_cartesian', ReachValues)
    reach_pose(effector_goal)

def arm_retract():
    rospy.wait_for_service('/mobile_manip/reach_name')
    reach_recorded_pose = rospy.ServiceProxy('/mobile_manip/reach_name', ReachName)
    reach_recorded_pose("retract")

# def reach_recorded():


#     state = recorded_pos.get()
#     rospy.wait_for_service('/mobile_manip/reach_name')
#     reach_recorded_pose = rospy.ServiceProxy('/mobile_manip/reach_name', ReachName)
#     reach_recorded_pose(state)

def activate_robot_arm():

    # position du tag dans le referentiel de la camera
    if (global_variables.tag_msg.detections == []):
        tag_x = 0.00
        tag_y = 0.00
        tag_z = 0.00
    else:
        tag_x = global_variables.tag_msg.detections[0].pose.pose.pose.position.x
        tag_y = global_variables.tag_msg.detections[0].pose.pose.pose.position.y
        tag_z = global_variables.tag_msg.detections[0].pose.pose.pose.position.z


    # transfo camera -> robot
    X_CAM = 0.1852
    Y_CAM = 0.024896
    Z_CAM = 0.45805 # should be 0.45805

    # matrice de transfo
    c_cr = th((X_CAM, Y_CAM, Z_CAM)) @ rh(R.from_euler('YZ', (90, -90), degrees=True))
    print(np.round(c_cr, 2))

    # coordonnees du tag dans le repere de la base du robot (mobile_manip/base_link)
    arm_pos = c_cr @ [tag_x, tag_y, tag_z, 1]

    arm_x = np.round(arm_pos[0], 2)
    arm_y = np.round(arm_pos[1], 2)
    arm_z = np.round(arm_pos[2], 2)

    # print(arm_x, arm_y, arm_z)

    print("Robot arm activated!")
    print(f"reaching position:\nx:{arm_x}\ny:{arm_y}\nz:{arm_z}")
    arm_reach_coord(arm_x, arm_y, arm_z)

def retract_robot_arm():
    print("Retracting robot arm")
    arm_retract()
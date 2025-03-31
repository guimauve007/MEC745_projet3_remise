import global_variables
import robot_positioning
import robot_control

import rospy
from lab_utils.plan_utils import *
from lab_utils.astart import AStarPlanner

from matplotlib import image as mpimg
from math import atan2, sqrt, pi
import numpy as np


# paramètres du suivi de trajectoire
#if global_variables.simu_status == True:

KP_ANGULAR = 0.4  # Gain proportionnel vitesse angulaire                val init (0.8)
KP_LINEAR = 0.005  
TOLERANCE_ANGLE = 0.04 # rad (val pre = 0.05) 
TOLERANCE_DISTANCE = 0.6 # m (val pre = 0.3) 
VITESSE_LINEAIRE_ANGULAIRE= 0.1

#else:
# KP_ANGULAR = 0.9  # Gain proportionnel vitesse angulaire                val init (0.8)
# KP_LINEAR = 0.01  # Gain proportionnel vitesse linéaire                 val init (0.04)
# TOLERANCE_ANGLE = 0.04 # rad (val pre = 0.05) 
# TOLERANCE_DISTANCE = 0.6 # m (val pre = 0.3) 
# VITESSE_LINEAIRE_ANGULAIRE = 0.05


#MIN_LINEAR_SPEED = 0.0 # m/s            val init 0.15
# MAX_LINEAR_SPEED = 0.5 # m/s
# MIN_ANGULAR_SPEED = 0.2 # rad/s
# MAX_ANGULAR_SPEED = 0.6 # rad/s

# pas pir angle 0.08 et tol_dist 0.5
def robot_pathing_init():
    # Global variables init
    global_variables.moveToDestination = False
    global_variables.createPath = False
    global_variables.waypoint_index = 0
    global_variables.currentPath = None

    # path planning
    image = mpimg.imread("/home/jupyter-mecbotg11/Project5/Maps/a2230_map_closed.png")
    map_img = 1-np.array(image[:,:,1])
    mat_map = map_img
    map = BMPMap(width=map_img.shape[1], height=map_img.shape[0], mat=mat_map)
    global_variables.astarPlanner = AStarPlanner(map=map, step_size=10, collision_radius=20, heuristic_dist='Manhattan')

def set_move_to_destination(state):
    global_variables.moveToDestination = state

def set_create_path(state):
    global_variables.createPath = state

def get_waypoint_distance(x1, y1, x2, y2):
    """retourne la position entre le robot et le waypoint"""
    
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)

def set_destination(x, y):
    global_variables.x_destination = x
    global_variables.y_destination = y

def create_path(x, y):
    """Crée la trajectoire"""
    
    start_x, start_y, theta = robot_positioning.get_robot_map_pixel_position()
    start = Point(start_x+robot_positioning.get_robot_map_offset_x(), -start_y+robot_positioning.get_robot_map_offset_y())
    end = Point(x, y)

    print(f"start x: {start.x}, end x: {end.x}")
    print(f"start y: {start.y}, end y: {end.y}")

    try:
        global_variables.astarPlanner.plan(start, end)
        global_variables.currentPath = global_variables.astarPlanner.finalPath
    except ValueError:
        print("Cannot reach destination, try again!")
        exit_pathing_mode()
        return  # Exit function safely

    for i in range(len(global_variables.astarPlanner.finalPath)-1):
        pt = global_variables.astarPlanner.finalPath[i].tuple()
        print(pt) # Imprime la liste de positions du trajet

def exit_pathing_mode():
    global_variables.waypoint_index = 0
    global_variables.moveToDestination = False
    global_variables.currentPath = None
    robot_control.stop_robot()

def force_exit_pathing_mode():
    exit_pathing_mode()
    print("Pathing mode was exited manually")

def follow_path():
    """suivi de trajectoire"""
       
    # position actuelle du robot
    if global_variables.waypoint_index != len(global_variables.currentPath) :

        waypoint_x=global_variables.currentPath[global_variables.waypoint_index].x - robot_positioning.get_robot_map_offset_x()
        waypoint_y=-(global_variables.currentPath[global_variables.waypoint_index].y - robot_positioning.get_robot_map_offset_y())

        # position actuelle du robot 
        robot_x, robot_y, robot_cap = robot_positioning.get_robot_map_pixel_position()

        waypoint_distance = get_waypoint_distance(robot_x, robot_y, waypoint_x, waypoint_y)
        waypoint_angle = robot_positioning.get_heading_error(robot_x, robot_y, robot_cap, waypoint_x, waypoint_y)

        if waypoint_distance <= TOLERANCE_DISTANCE / robot_positioning.get_pixel_to_meter_ratio():
            global_variables.waypoint_index +=1
            print(f"waypoint number: {global_variables.waypoint_index}")
            print(f"robot position: {robot_x}, {robot_y}")
            return

        linear = waypoint_distance*KP_LINEAR
            
        angular = waypoint_angle*KP_ANGULAR
        
        # Si l'angle est suffisamment proche de la cible, on peut se déplacer linéairement
        if abs(waypoint_angle) < TOLERANCE_ANGLE:
            #print("Le robot est correctement orienté, déplacement linéaire.")
            robot_control.move_robot(linear, angular) 
        else:
            #print(f"Le robot ajuste son orientation. Différence d'angle : {difference_angle}")
            robot_control.move_robot(VITESSE_LINEAIRE_ANGULAIRE, angular)  # Correction de l'orientation
            
        #rospy.sleep(0.1)

def process():
    if global_variables.createPath == True:
        create_path(global_variables.x_destination, global_variables.y_destination)
        global_variables.createPath = False

    if global_variables.moveToDestination == True:
        follow_path()
    
    if global_variables.waypoint_index > 0 and global_variables.waypoint_index == len(global_variables.currentPath):
        exit_pathing_mode()
        print("Robot reached it's destination!")
    
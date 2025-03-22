import rospy
from mobile_manip.srv import ReachName, GetValues, ReachValues
import tkinter as tk
import os, sys
current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from lab_utils.ip_config import get_ip
from os import environ


# *************************************************************************************************

VOTRE_NUMERO_EQUIPE = 1
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


def reach_coord():
    effector_goal = [float(x.get()), float(y.get()), float(z.get()), float(arm_x.get()), float(arm_y.get()), float(arm_z.get()), float(wall.get())]
    rospy.wait_for_service('/mobile_manip/reach_cartesian')
    reach_pose = rospy.ServiceProxy('/mobile_manip/reach_cartesian', ReachValues)
    reach_pose(effector_goal)

def reach_recorded():
    state = recorded_pos.get()
    rospy.wait_for_service('/mobile_manip/reach_name')
    reach_recorded_pose = rospy.ServiceProxy('/mobile_manip/reach_name', ReachName)
    reach_recorded_pose(state)

root = tk.Tk()

xyz = tk.Label(root, text='Cartesian coords')
xyz.grid(row=0, column=0)

x = tk.Entry(root)
x.insert(-1, 0.4)
x.grid(row=1, column=0)
y = tk.Entry(root)
y.insert(-1, 0)
y.grid(row=2, column=0)
z = tk.Entry(root)
z.insert(-1, 1.0)
z.grid(row=3, column=0)

arm_xyz = tk.Label(root, text='Effector orientation') # Z->Y'->X''
arm_xyz.grid(row=0, column=1)

arm_x = tk.Entry(root)
arm_x.insert(-1, 0)
arm_x.grid(row=1, column=1)
arm_y = tk.Entry(root)
arm_y.insert(-1, 0)
arm_y.grid(row=2, column=1)
arm_z = tk.Entry(root)
arm_z.insert(-1, 0)
arm_z.grid(row=3, column=1)

wall_label = tk.Label(root, text='Virtual wall offset')
wall_label.grid(row=4, column=0)
wall = tk.Entry(root)
wall.insert(-1, 0)
wall.grid(row=4, column=1)

btn_coord = tk.Button(root, text='Reach cartesian coords', command=reach_coord)
btn_coord.grid(row=0, column=2, rowspan=2)
btn_recorded = tk.Button(root, text='Reach recorded position', command=reach_recorded)
btn_recorded.grid(row=2, column=2, rowspan=2)

recorded_label = tk.Label(root, text='Recorded pos to reach (vertical, home, retract)')
recorded_label.grid(row=5, column=0, columnspan=2)
recorded_pos = tk.Entry(root)
recorded_pos.insert(-1, 'vertical')
recorded_pos.grid(row=5, column=2)

root.mainloop()
import rospy
from jackal_msgs.msg import Drive
from sensor_msgs.msg import Image
import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from lab_utils.ip_config import get_ip
from os import environ
from tag_detection import *


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


# Camera subscriber callback
cam_msg = Image()
def cam_callback(msg):
    global cam_msg
    cam_msg = msg

# ROS subscribers et publishers
if SIM:
    cmd_drive_pub = rospy.Publisher('/mobile_manip/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
else:
    cmd_drive_pub = rospy.Publisher('/mobile_manip/base/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
cam_sub = rospy.Subscriber('/mobile_manip/d435i/color/image_raw', Image, cam_callback)
init_tag_detection()


## DÉPLACEMENT DU ROBOT
interwheel_distance = 0.434
left_wheel_radius = 0.049
right_wheel_radius = 0.049
linear_speed = 0.5
angular_speed = 0.5
def move_robot(linear, angular):
    vel_left  = (linear - angular * interwheel_distance / 2.0) / left_wheel_radius
    vel_right = (linear + angular * interwheel_distance / 2.0) / right_wheel_radius

    # Envoi des commandes au roues par topic ROS
    cmd_drive_msg = Drive()
    cmd_drive_msg.drivers[0] = vel_left
    cmd_drive_msg.drivers[1] = vel_right
    cmd_drive_pub.publish(cmd_drive_msg)


## INTERFACE
from cv_bridge import CvBridge
import tkinter as tk
from PIL import Image, ImageTk

def resize_image(image, max_width, max_height):
    width, height = image.size
    aspect_ratio = width / height
    if width > max_width or height > max_height:
        if width > height:
            new_width = max_width
            new_height = int(max_width / aspect_ratio)
        else:
            new_height = max_height
            new_width = int(max_height * aspect_ratio)
        return image.resize((new_width, new_height)) 
    else:
        return image

bridge = CvBridge()
def update_plot():
    if (cam_msg.height == 0):
        pass
    else:
        cv_image = bridge.imgmsg_to_cv2(cam_msg, "rgb8")
        image = Image.fromarray(cv_image)
        image = resize_image(image, 960, 540)
        imageTk = ImageTk.PhotoImage(image=image)
        label.configure(image=imageTk)
        label.image = imageTk
        updateTagDetectionData(x_value, y_value, z_value, orientation_value)
    root.after(10, update_plot)  

root = tk.Tk()

# Main display
label = tk.Label(root)
label.pack()
buttons_frame = tk.Frame(root)
buttons_frame.pack()

btn_forward = tk.Button(buttons_frame, text="Forward", command=lambda: move_robot(linear_speed, 0))
btn_left = tk.Button(buttons_frame, text="Left", command=lambda: move_robot(0, angular_speed))
btn_backward = tk.Button(buttons_frame, text="Backward", command=lambda: move_robot(-linear_speed, 0))
btn_right = tk.Button(buttons_frame, text="Right", command=lambda: move_robot(0, -angular_speed))
btn_stop = tk.Button(buttons_frame, text="Stop", command=lambda: move_robot(0,0))

btn_forward.grid(row=0, column=1)
btn_left.grid(row=1, column=0)
btn_stop.grid(row=1, column=1)
btn_right.grid(row=1, column=2)
btn_backward.grid(row=2, column=1)

# Tag display
x_value = tk.DoubleVar()
y_value = tk.DoubleVar()
z_value = tk.DoubleVar()
orientation_value = tk.DoubleVar()

title_label = tk.Label(root, text="Tag detection")
title_label.grid(row=0, column=0, columnspan=3)

x_label = tk.Label(root, text="x:")
x_label.grid(row=1, column=0)
y_label = tk.Label(root, text="y:")
y_label.grid(row=2, column=0)
z_label = tk.Label(root, text="z:")
z_label.grid(row=3, column=0)
orientation_label = tk.Label(root, text="Orientation:")
orientation_label.grid(row=4, column=0)

x_value_label = tk.Label(root, textvariable=x_value)
x_value_label.grid(row=1, column=1)
y_value_label = tk.Label(root, textvariable=y_value)
y_value_label.grid(row=2, column=1)
z_value_label = tk.Label(root, textvariable=z_value)
z_value_label.grid(row=3, column=1)
orientation_value_label = tk.Label(root, textvariable=orientation_value)
orientation_value_label.grid(row=4, column=1)

update_plot()
root.mainloop()

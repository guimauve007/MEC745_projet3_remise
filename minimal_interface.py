import rospy
from jackal_msgs.msg import Drive
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from lab_utils.ip_config import get_ip
from os import environ
from scipy.spatial.transform import Rotation as R
import numpy as np
from cv_bridge import CvBridge
import tkinter as tk
from PIL import Image, ImageTk

# *************************************************************************************************
VOTRE_NUMERO_EQUIPE = 1
SIM = True
ADMIN = False
VOTRE_NUMERO_ROBOT = 7
    
if SIM:
    environ['ROS_MASTER_URI'] = f"http://localhost:113{70+VOTRE_NUMERO_EQUIPE if not ADMIN else 11}/"
else:
    environ['ROS_MASTER_URI'] = "http://cpr-ets05-0{}.local:11311/".format(VOTRE_NUMERO_ROBOT)
    environ['ROS_IP'] = get_ip()

rospy.init_node('dingo_controller', anonymous=True)
rate = rospy.Rate(50)

# *************************************************************************************************

# Global variables for ROS topics
cam_msg = Image()
tag_array = AprilTagDetectionArray()

# Camera subscriber callback
def cam_callback(msg):
    global cam_msg
    cam_msg = msg

# Apriltag detection callback
def tag_callback(msg):
    global tag_array
    tag_array = msg

# Function to extract heading from quaternion
def get_heading_from_quaternion(q):
    if(np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2) == 0):
        print('Quaternion norm 0 : ', q.x, q.y, q.z, q.w)
        return 0
    else:
        r = R.from_quat([q.x, q.y, q.z, q.w])
        angles = r.as_euler('xyz', degrees=False)
        return angles[2]

# ROS Publishers & Subscribers
if SIM:
    cmd_drive_pub = rospy.Publisher('/mobile_manip/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
else:
    cmd_drive_pub = rospy.Publisher('/mobile_manip/base/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
cam_sub = rospy.Subscriber('/mobile_manip/d435i/color/image_raw', Image, cam_callback)
tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)

# Robot movement function
def move_robot(linear, angular):
    interwheel_distance = 0.434
    wheel_radius = 0.049
    vel_left  = (linear - angular * interwheel_distance / 2.0) / wheel_radius
    vel_right = (linear + angular * interwheel_distance / 2.0) / wheel_radius

    cmd_drive_msg = Drive()
    cmd_drive_msg.drivers[0] = vel_left
    cmd_drive_msg.drivers[1] = vel_right
    cmd_drive_pub.publish(cmd_drive_msg)

# Tkinter UI setup
root = tk.Tk()
label = tk.Label(root)
label.pack()

buttons_frame = tk.Frame(root)
buttons_frame.pack()

btn_forward = tk.Button(buttons_frame, text="Forward", command=lambda: move_robot(0.5, 0))
btn_left = tk.Button(buttons_frame, text="Left", command=lambda: move_robot(0, 0.5))
btn_backward = tk.Button(buttons_frame, text="Backward", command=lambda: move_robot(-0.5, 0))
btn_right = tk.Button(buttons_frame, text="Right", command=lambda: move_robot(0, -0.5))
btn_stop = tk.Button(buttons_frame, text="Stop", command=lambda: move_robot(0, 0))

btn_forward.grid(row=0, column=1)
btn_left.grid(row=1, column=0)
btn_stop.grid(row=1, column=1)
btn_right.grid(row=1, column=2)
btn_backward.grid(row=2, column=1)

# Tag Detection UI elements
title_label = tk.Label(root, text="Tag detection")
title_label.pack()

x_value = tk.DoubleVar()
y_value = tk.DoubleVar()
z_value = tk.DoubleVar()
orientation_value = tk.DoubleVar()

def update_tag_display():
    if not tag_array.detections:
        x_value.set(0.00)
        y_value.set(0.00)
        z_value.set(0.00)
        orientation_value.set(0.00)
    else:
        x_value.set(tag_array.detections[0].pose.pose.pose.position.x)
        y_value.set(tag_array.detections[0].pose.pose.pose.position.y)
        z_value.set(tag_array.detections[0].pose.pose.pose.position.z)
        orientation_value.set(get_heading_from_quaternion(tag_array.detections[0].pose.pose.pose.orientation))
    root.after(10, update_tag_display)

x_label = tk.Label(root, text="x:")
y_label = tk.Label(root, text="y:")
z_label = tk.Label(root, text="z:")
orientation_label = tk.Label(root, text="Orientation:")

x_label.pack()
x_display = tk.Label(root, textvariable=x_value)
x_display.pack()
y_label.pack()
y_display = tk.Label(root, textvariable=y_value)
y_display.pack()
z_label.pack()
z_display = tk.Label(root, textvariable=z_value)
z_display.pack()
orientation_label.pack()
orientation_display = tk.Label(root, textvariable=orientation_value)
orientation_display.pack()

# Camera display bridge
bridge = CvBridge()
def update_camera_display():
    if cam_msg.height == 0:
        pass
    else:
        cv_image = bridge.imgmsg_to_cv2(cam_msg, "rgb8")
        image = Image.fromarray(cv_image)
        image = image.resize((960, 540))
        imageTk = ImageTk.PhotoImage(image=image)
        label.configure(image=imageTk)
        label.image = imageTk
    root.after(10, update_camera_display)

# Start updating UI
title_label.pack()
update_camera_display()
update_tag_display()
root.mainloop()

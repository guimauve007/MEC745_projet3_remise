import rospy
from jackal_msgs.msg import Drive
from sensor_msgs.msg import Image
import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from lab_utils.ip_config import get_ip
from os import environ
from tag_detection import *

#path planning includes
from matplotlib import image as mpimg
from lab_utils.plan_utils import *
from lab_utils.astart import AStarPlanner

from nav_msgs.msg import Odometry


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

# Global variables
# Definir les variables globales ici

# Camera subscriber callback
cam_msg = Image()
def cam_callback(msg):
    global cam_msg
    cam_msg = msg

# Odometry subscriber callback
odom_msg = Odometry()
def odometry_callback(msg):
    global odom_msg
    odom_msg = msg
    # print(f"position x: {msg.pose.pose.position.x}, position y: {msg.pose.pose.position.y}")

# ROS subscribers et publishers
if SIM:
    cmd_drive_pub = rospy.Publisher('/mobile_manip/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
else:
    cmd_drive_pub = rospy.Publisher('/mobile_manip/base/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
cam_sub = rospy.Subscriber('/mobile_manip/d435i/color/image_raw', Image, cam_callback)
odometry_sub = rospy.Subscriber('/odometry/filtered', Odometry, odometry_callback)
init_tag_detection()


## UPDATE D'AFFICHAGE (a mettre dans un autre fichier .py?)
def updateTargetDestinationDisplay(x_clicked, y_clicked):
    x_clicked_value.set('{:2f}'.format(x_clicked))
    y_clicked_value.set('{:2f}'.format(y_clicked))

## CONVERSIONS (a mettre dans un autre fichier .py?)
def convert_pixel_pos_to_distance(x_clicked, y_clicked):
    # Convertir de position de pixel a une distance en m
    x_distance = 0.02*x_clicked
    y_distance = 0.02*y_clicked
    
    return x_distance, y_distance

def convert_pos_to_pixel_distance(x_meter, y_meter):
    # Convertir de distance en m a position de pixel
    # en ajoutant le offset vu que le 0,0 du pixel est en haut a gauche de la map
    x_pixel = x_meter/0.02 + 1444.0
    y_pixel = y_meter/0.02 + 1056.0
    
    return x_pixel, y_pixel

def distance_to_linear_angular(x_distance, y_distance):
    # Convertir une distance en X Y en ratio de linear et angular a envoyer a move_robot()?

    return

# path planning

image = mpimg.imread("Maps/a2230_map_closed.png")

map_img = 1-np.array(image[:,:,1])
print(map_img.shape)
mat_map = map_img
map = BMPMap(width=map_img.shape[1], height=map_img.shape[0], mat=mat_map)

#astarPlanner = AStarPlanner(map=map, step_size=10, collision_radius=12, heuristic_dist='Euclidean')
astarPlanner = AStarPlanner(map=map, step_size=10, collision_radius=12, heuristic_dist='Euclidean')

def get_position_in_meters():
    #odometry_msg = Odometry()

    # http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
    # odometry_msg.header.frame_id = "odom"
    # odometry_msg.header.stamp = rospy.Time.now()
    # odometry_msg.child_frame_id = "base_link"
    # odometry_msg.pose.pose = Pose(Point(*pose), Quaternion(*orientation))
    # odometry_msg.twist.twist = Twist(Vector3(vlin, 0, 0), Vector3(0, 0, vang))
    # print(odometry_msg.pose.pose)
    return odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y

def create_path(x, y):
    
    start_x_meter, start_y_meter = get_position_in_meters()
    start_x, start_y = convert_pos_to_pixel_distance(start_x_meter, start_y_meter)
    start = Point(start_x, start_y)
    end = Point(x, y)
    print(f"start x: {start.x}, end x: {end.x}")
    print(f"start y: {start.y}, end y: {end.y}")
    astarPlanner.plan(start=start, target=end)
    #astarPlanner.plan(Point(1444.7, 1052.1), Point(1750, 740))

    for i in range(len(astarPlanner.finalPath)-1):
        pt = astarPlanner.finalPath[i].tuple()
        print(pt)



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
        updateTagDetectionDisplay(x_value, y_value, z_value, orientation_value)
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

btn_forward.pack()
btn_left.pack(side=tk.LEFT)
btn_stop.pack(side=tk.LEFT)
btn_right.pack(side=tk.LEFT)
btn_backward.pack()

# Load map image
map_image = Image.open("Maps/a2230_map_closed.png")
map_image = map_image.resize((500, 300))
map_photo = ImageTk.PhotoImage(map_image)

# Canvas for map
def on_map_click(event):
    x_clicked, y_clicked = event.x, event.y
    print(f"Clicked at: ({x_clicked}, {y_clicked})")
    updateTargetDestinationDisplay(x_clicked, y_clicked)
    #move_robot(distance_to_linear_angular(convert_pixel_pos_to_distance()))  
    #move_robot(0.5, 0) # Example movement command
    create_path(x_clicked, y_clicked)

canvas = tk.Canvas(root, width=500, height=300)
canvas.create_image(0, 0, anchor=tk.NW, image=map_photo)
canvas.bind("<Button-1>", on_map_click)
canvas.pack()

# Tag Detection Display
tag_frame = tk.Frame(root)

x_value = tk.DoubleVar()
y_value = tk.DoubleVar()
z_value = tk.DoubleVar()
orientation_value = tk.DoubleVar()

# Tag Detection Label
tk.Label(tag_frame, text="Tag Detection").pack()

# Create a frame for each label-text pair
x_frame = tk.Frame(tag_frame)
tk.Label(x_frame, text="x:").pack(side="left")
tk.Label(x_frame, textvariable=x_value).pack(side="left")
x_frame.pack()

y_frame = tk.Frame(tag_frame)
tk.Label(y_frame, text="y:").pack(side="left")
tk.Label(y_frame, textvariable=y_value).pack(side="left")
y_frame.pack()

z_frame = tk.Frame(tag_frame)
tk.Label(z_frame, text="z:").pack(side="left")
tk.Label(z_frame, textvariable=z_value).pack(side="left")
z_frame.pack()

orientation_frame = tk.Frame(tag_frame)
tk.Label(orientation_frame, text="Orientation:").pack(side="left")
tk.Label(orientation_frame, textvariable=orientation_value).pack(side="left")
orientation_frame.pack()

tag_frame.pack()

# Target destination display (north east position)
target_frame = tk.Frame(root)

x_clicked_value = tk.DoubleVar()
y_clicked_value = tk.DoubleVar()


# Target Destination Label
tk.Label(target_frame, text="Target Destination").pack()

# Create a frame for each label-text pair
x_clicked_frame = tk.Frame(target_frame)
tk.Label(x_clicked_frame, text="x_clicked:").pack(side="left")
tk.Label(x_clicked_frame, textvariable=x_clicked_value).pack(side="left")
x_clicked_frame.pack()

y_clicked_frame = tk.Frame(target_frame)
tk.Label(y_clicked_frame, text="y_clicked:").pack(side="left")
tk.Label(y_clicked_frame, textvariable=y_clicked_value).pack(side="left")
y_clicked_frame.pack()

target_frame.pack()

update_plot()
root.mainloop()

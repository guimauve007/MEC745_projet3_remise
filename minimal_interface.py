import rospy
from math import atan2, sqrt, pi
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

# paramètres du suivi de trajectoire
KP_ANGULAR = 1  # Gain proportionnel vitesse angulaire
KP_LINEAR = 0.04  # Gain proportionnel vitesse linéaire
MIN_LINEAR_SPEED = 0.15 # m/s
MAX_LINEAR_SPEED = 0.5 # m/s
MIN_ANGULAR_SPEED = 0.05 # rad/s
MAX_ANGULAR_SPEED = 0.5 # rad/s
TOLERANCE_ANGLE = 0.05 # rad
TOLERANCE_DISTANCE = 0.05 # m

# paramètres du robot
INTERWHEEL_DISTANCE = 0.434 # m
LEFT_WHEEL_RADIUS = 0.049 # m
RIGHT_WHEEL_RADIUS = 0.049 # m

# Vitesses pour le déplacement manuel
MANUAL_LINEAR_SPEED = 0.5 # m/s
MANUAL_ANGULAR_SPEED = 0.5 # rad/s

# Paramètres de la map
MAP_SCALING = 1/8
PIXEL_TO_METER = 0.02 # mètres par pixel
ROBOT_MAP_OFFSET_X = 1444.7 # position en X du robot dans la map d'origine (en pixels)
ROBOT_MAP_OFFSET_Y = 1052.1 # position  en Y du robot dans la map d'origine (en pixels)


# path planing
image = mpimg.imread("/home/jupyter-mecbotg11/Project5/Maps/a2230_map_closed.png")
map_img = 1-np.array(image[:,:,1])
mat_map = map_img
map = BMPMap(width=map_img.shape[1], height=map_img.shape[0], mat=mat_map)
astarPlanner = AStarPlanner(map=map, step_size=10, collision_radius=20, heuristic_dist='Manhattan')
    
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


# Update display
def updateTargetDestinationDisplay(x_clicked, y_clicked):
    x_clicked_value.set('{:2f}'.format(x_clicked))
    y_clicked_value.set('{:2f}'.format(y_clicked))


def convert_pos_to_pixel_distance(x_meter, y_meter):
    """Conversion de distance en m à pixel"""

    x_pixel = x_meter/PIXEL_TO_METER
    y_pixel = y_meter/PIXEL_TO_METER
    
    return x_pixel, y_pixel


def get_robot_map_pixel_position():
    """ retourne la position du robot dans la map dorigine """
    
    robot_x, robot_y = convert_pos_to_pixel_distance(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y)
    q = odom_msg.pose.pose.orientation  # Quaternion
    theta = get_heading_from_quaternion(q)

    return robot_x, robot_y, theta


def create_path(x, y):
    """Crée la trajectoire"""
    
    start_x, start_y, theta = get_robot_map_pixel_position()
    start = Point(start_x+ROBOT_MAP_OFFSET_X, start_y+ROBOT_MAP_OFFSET_Y)
    end = Point(x, y)
    print(f"start x: {start.x}, end x: {end.x}")
    print(f"start y: {start.y}, end y: {end.y}")

    astarPlanner.plan(start, end)

    for i in range(len(astarPlanner.finalPath)-1):
        pt = astarPlanner.finalPath[i].tuple()
        print(pt) # Imprime la liste de positions du trajet


def get_heading_error(current_x, current_y, current_theta, target_x, target_y):
    """retourne l'erreur de heading"""
    
    target_theta = atan2(target_y - current_y, target_x - current_x)
    heading_error = target_theta - current_theta
    
    return heading_error


def get_waypoint_distance(x1, y1, x2, y2):
    """retourne la position entre le robot et le waypoint"""
    
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)


# Fonction pour le calcul de l'orientation à partir d'un quaternion
# Entrée : Quaternion [x, y ,z ,w]
# Sortie : Angle de lacet (yaw) en radians
def get_heading_from_quaternion(q):
    """retourne le heading du robot en rad (-pi à pi)"""
    
    r = R.from_quat([q.x, q.y, q.z, q.w])
    angles = r.as_euler('xyz', degrees=False)
    return angles[2]


def follow_path(path):
    """suivi de trajectoire"""
       
    # position actuelle du robot
    waypoint_index = 0
    while waypoint_index != len(path) :

        waypoint_x=path[waypoint_index].x - ROBOT_MAP_OFFSET_X
        waypoint_y=-(path[waypoint_index].y - ROBOT_MAP_OFFSET_Y)

        # position actuelle du robot 
        robot_x, robot_y, robot_cap = get_robot_map_pixel_position()

        waypoint_distance = get_waypoint_distance(robot_x, robot_y, waypoint_x, waypoint_y)
        waypoint_angle = get_heading_error(robot_x, robot_y, robot_cap, waypoint_x, waypoint_y)

        if waypoint_distance < TOLERANCE_DISTANCE / PIXEL_TO_METER:
            waypoint_index +=1
            print(f"waypoint number: {waypoint_index}")
            print(f"robot position: {robot_x}, {robot_y}")
            continue

        linear = min(waypoint_distance*KP_LINEAR, MIN_LINEAR_SPEED)
            
        angular = waypoint_angle*KP_ANGULAR
        
        # Si l'angle est suffisamment proche de la cible, on peut se déplacer linéairement
        if abs(waypoint_angle) < TOLERANCE_ANGLE:
            #print("Le robot est correctement orienté, déplacement linéaire.")
            move_robot(linear, angular)
        else:
            #print(f"Le robot ajuste son orientation. Différence d'angle : {difference_angle}")
            move_robot(0, angular)  # Correction de l'orientation
            
        rospy.sleep(0.1)
        

def move_robot(linear_speed, angular_speed):
    vel_left  = (linear_speed - angular_speed * INTERWHEEL_DISTANCE / 2.0) / LEFT_WHEEL_RADIUS
    vel_right = (linear_speed + angular_speed * INTERWHEEL_DISTANCE / 2.0) / RIGHT_WHEEL_RADIUS

    # Envoi des commandes au roues par topic ROS
    cmd_drive_msg = Drive()
    cmd_drive_msg.drivers[0] = vel_left
    cmd_drive_msg.drivers[1] = vel_right
    cmd_drive_pub.publish(cmd_drive_msg)
    #update_plot()


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

btn_forward = tk.Button(buttons_frame, text="Forward", command=lambda: move_robot(MANUAL_LINEAR_SPEED, 0))
btn_left = tk.Button(buttons_frame, text="Left", command=lambda: move_robot(0, MANUAL_ANGULAR_SPEED))
btn_backward = tk.Button(buttons_frame, text="Backward", command=lambda: move_robot(-MANUAL_LINEAR_SPEED, 0))
btn_right = tk.Button(buttons_frame, text="Right", command=lambda: move_robot(0, -MANUAL_ANGULAR_SPEED))
btn_stop = tk.Button(buttons_frame, text="Stop", command=lambda: move_robot(0,0))

btn_forward.pack()
btn_left.pack(side=tk.LEFT)
btn_stop.pack(side=tk.LEFT)
btn_right.pack(side=tk.LEFT)
btn_backward.pack()

# Load map image
map_image = Image.open("/home/jupyter-mecbotg11/Project5/Maps/a2230_map_closed.png")
map_image = map_image.resize((int(map_image.width * MAP_SCALING), int(map_image.height * MAP_SCALING)))  # Resize the image
map_photo = ImageTk.PhotoImage(map_image)

# Canvas for map
def on_map_click(event):
    x_clicked, y_clicked = event.x, event.y
    print(f"Clicked at: ({x_clicked}, {y_clicked})")
    updateTargetDestinationDisplay(x_clicked, y_clicked)
    x_click_scaled = x_clicked/MAP_SCALING
    y_click_scaled = y_clicked/MAP_SCALING
    create_path(x_click_scaled, y_click_scaled)
    follow_path(astarPlanner.finalPath)
    

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

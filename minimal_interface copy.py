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

# Global variables
# Definir les variables globales ici
# Control gains
Kp_angular = 1  # Proportional gain for angular velocity
Kp_linear = 0.04  # Proportional gain for linear velocity
min_linear_speed = 0.75 # Minimum speed for slow approach
MAX_LINEAR_SPEED = 0.5  # Maximum forward speed
MAX_ANGULAR_SPEED = 0.5
angular_speed_limit = 1  # Limit for angular velocity
linear_velocity = 0
angular_speed = 0

scaling = 1/8
origin_x = 1444.7
origin_y = 1052.1

map_offset_x = origin_x * scaling
map_offset_y = origin_y * scaling

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
    x_pixel = x_meter/0.02
    y_pixel = y_meter/0.02
    
    return x_pixel, y_pixel



def get_robot_map_pixel_position():
    #odometry_msg = Odometry()

    # http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
    # odometry_msg.header.frame_id = "odom"
    # odometry_msg.header.stamp = rospy.Time.now()
    # odometry_msg.child_frame_id = "base_link"
    # odometry_msg.pose.pose = Pose(Point(*pose), Quaternion(*orientation))
    # odometry_msg.twist.twist = Twist(Vector3(vlin, 0, 0), Vector3(0, 0, vang))
    # print(odometry_msg.pose.pose)
    robot_x, robot_y = convert_pos_to_pixel_distance(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y)

    q = odom_msg.pose.pose.orientation  # Quaternion
    theta = get_heading_from_quaternion(q)
    # print(f"robot position:{robot_x}, {robot_y}, {theta}")
    return robot_x, robot_y, theta

def create_path(x, y):

    start_x, start_y, theta = get_robot_map_pixel_position()
    start = Point(start_x+origin_x, start_y+origin_y)
    end = Point(x, y)
    print(f"start x: {start.x}, end x: {end.x}")
    print(f"start y: {start.y}, end y: {end.y}")

    astarPlanner.plan(start, end)

    for i in range(len(astarPlanner.finalPath)-1):
        pt = astarPlanner.finalPath[i].tuple()
        print(pt)


def compute_heading_error(current_x, current_y, current_theta, target_x, target_y):
    """ Compute the error in heading (angular error) towards the target. """
    desired_theta = atan2(target_y - current_y, target_x - current_x)
    heading_error = desired_theta - current_theta
    
    return heading_error

def compute_distance(x1, y1, x2, y2):
    """ Compute Euclidean distance between two points. """
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)

# Fonction pour le calcul de l'orientation à partir d'un quaternion
# Entrée : Quaternion [x, y ,z ,w]
# Sortie : Angle de lacet (yaw) en radians
def get_heading_from_quaternion(q):
    r = R.from_quat([q.x, q.y, q.z, q.w])
    angles = r.as_euler('xyz', degrees=False)
    return angles[2]


def follow_path(path):
    #Paramètres

    tolerance_angle = 0.05

    linear_min = 0.15

    #print(liste_traj2)
    #position reel du robot 
    i = 0
    while i != len(path) :

        pos_x=path[i].x - origin_x
        pos_y=-(path[i].y - origin_y)

        #position reel du robot 
        robot_x, robot_y, robot_cap = get_robot_map_pixel_position()

        #print(cap_du_robot)

        deplacement_x= pos_x-robot_x
        deplacement_y= pos_y-robot_y

        if np.sqrt(deplacement_x**2+deplacement_y**2) < 0.05 /0.02:
            i +=1
            print(f"waypoint number: {i}")
            print(f"robot position: {robot_x}, {robot_y}")
            continue

        linear = np.sqrt(deplacement_x**2+deplacement_y**2)*Kp_linear
        if linear < linear_min:
            linear = linear_min
            
        angular = (np.arctan2(deplacement_y,deplacement_x) - robot_cap)*Kp_angular
            
        #Comparaison des angles
        # difference_angle = robot_cap-angular
        # angle_cible = np.arctan2(deplacement_y,deplacement_x)
        difference_angle = np.arctan2(deplacement_y,deplacement_x) - robot_cap
        
        #print(f"Cap du robot : {robot_cap}, Angle cible : {angle_cible}, Différence d'angle : {difference_angle}")

        # Si l'angle est suffisamment proche de la cible, on peut se déplacer linéairement
        if abs(difference_angle) < tolerance_angle:
            #print("Le robot est correctement orienté, déplacement linéaire.")
            move_robot(linear, angular)
        else:
            #print(f"Le robot ajuste son orientation. Différence d'angle : {difference_angle}")
            move_robot(0, angular)  # Correction de l'orientation
            
        #print (linear)
        rospy.sleep(0.1)
        

## DÉPLACEMENT DU ROBOT
# mesures
interwheel_distance = 0.434
left_wheel_radius = 0.049
right_wheel_radius = 0.049
# valeurs de vitesses pour les boutons
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
map_image = Image.open("/home/jupyter-mecbotg11/Project5/Maps/a2230_map_closed.png")
map_image = map_image.resize((int(map_image.width * scaling), int(map_image.height * scaling)))  # Resize the image
map_photo = ImageTk.PhotoImage(map_image)

# Canvas for map
def on_map_click(event):
    x_clicked, y_clicked = event.x, event.y
    print(f"Clicked at: ({x_clicked}, {y_clicked})")
    updateTargetDestinationDisplay(x_clicked, y_clicked)
    #move_robot(distance_to_linear_angular(convert_pixel_pos_to_distance()))  
    #move_robot(0.5, 0) # Example movement command
    x_click_scaled = x_clicked/scaling
    y_click_scaled = y_clicked/scaling
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

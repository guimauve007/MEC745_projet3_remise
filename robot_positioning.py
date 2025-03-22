
import global_variables

from math import atan2, sqrt, pi
from scipy.spatial.transform import Rotation as R

PIXEL_TO_METER = 0.02 # mètres par pixel
ROBOT_MAP_OFFSET_X = 1444.7 # position en X du robot dans la map d'origine (en pixels)
ROBOT_MAP_OFFSET_Y = 1052.1 # position en Y du robot dans la map d'origine (en pixels)

def get_robot_map_offset_x():
    return ROBOT_MAP_OFFSET_X

def get_robot_map_offset_y():
    return ROBOT_MAP_OFFSET_Y

# Fonction pour le calcul de l'orientation à partir d'un quaternion
# Entrée : Quaternion [x, y ,z ,w]
# Sortie : Angle de lacet (yaw) en radians
def get_heading_from_quaternion(q):
    """retourne le heading du robot en rad (-pi à pi)"""
    
    r = R.from_quat([q.x, q.y, q.z, q.w])
    angles = r.as_euler('xyz', degrees=False)
    return angles[2]

def get_heading_error(current_x, current_y, current_theta, target_x, target_y):
    """retourne l'erreur de heading"""
    
    target_theta = atan2(target_y - current_y, target_x - current_x)
    heading_error = target_theta - current_theta
    
    return heading_error

def convert_pos_to_pixel_distance(x_meter, y_meter):
    """Conversion de distance en m à pixel"""

    x_pixel = x_meter/PIXEL_TO_METER
    y_pixel = y_meter/PIXEL_TO_METER
    
    return x_pixel, y_pixel

def get_robot_map_pixel_position():
    """ retourne la position du robot dans la map dorigine """
    
    robot_x, robot_y = convert_pos_to_pixel_distance(global_variables.odom_msg.pose.pose.position.x, global_variables.odom_msg.pose.pose.position.y)
    q = global_variables.odom_msg.pose.pose.orientation  # Quaternion
    theta = get_heading_from_quaternion(q)

    return robot_x, robot_y, theta

def get_pixel_to_meter_ratio():
    return PIXEL_TO_METER


import global_variables

import numpy as np
from scipy.spatial.transform import Rotation as R



# *************************************************************************************************

# VOTRE_NUMERO_EQUIPE = 11
# SIM = True
# ADMIN = False
# VOTRE_NUMERO_ROBOT = 7
    
# if SIM:
#     environ['ROS_MASTER_URI'] = f"http://localhost:113{70+VOTRE_NUMERO_EQUIPE if not ADMIN else 11}/"
# else:
#     environ['ROS_MASTER_URI'] = "http://cpr-ets05-0{}.local:11311/".format(VOTRE_NUMERO_ROBOT)
#     environ['ROS_IP'] = get_ip() # adresse IP de votre station de travail

# rospy.init_node('dingo_controller', anonymous=True)
# rate = rospy.Rate(50)

# *************************************************************************************************

def get_heading_from_quaternion(q):
    if(np.sqrt(q.x**2+q.y**2+q.z**2+q.w**2) == 0):
        print('Quaternion norm 0 : ', q.x, q.y, q.z, q.w)
        return 0
    else:
        r = R.from_quat([q.x, q.y, q.z, q.w])
        angles = r.as_euler('xyz', degrees=False)
        return angles[2]

def updateTagDetectionDisplay(x_value, y_value, z_value, orientation_value):
    if (global_variables.tag_msg.detections == []):
        x_value.set(0.00)
        y_value.set(0.00)
        z_value.set(0.00)
        orientation_value.set(0.00)
    else: # Affiche le premier tag de la liste detections
        if global_variables.tag_msg.detections[0].pose.pose.pose.position.x != []:
            x_value.set('{:2f}'.format(global_variables.tag_msg.detections[0].pose.pose.pose.position.x))
        if global_variables.tag_msg.detections[0].pose.pose.pose.position.y != []:
            y_value.set('{:2f}'.format(global_variables.tag_msg.detections[0].pose.pose.pose.position.y))
        if global_variables.tag_msg.detections[0].pose.pose.pose.position.z != []:
            z_value.set('{:2f}'.format(global_variables.tag_msg.detections[0].pose.pose.pose.position.z))
        if global_variables.tag_msg.detections[0].pose.pose.pose.orientation != []:
            orientation_value.set('{:2f}'.format(get_heading_from_quaternion(global_variables.tag_msg.detections[0].pose.pose.pose.orientation)))
        
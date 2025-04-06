import global_variables

import numpy as np
from scipy.spatial.transform import Rotation as R

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
    else: # Show the first tag in the detection list
        if global_variables.tag_msg.detections[0].pose.pose.pose.position.x != []:
            x_value.set('{:2f}'.format(global_variables.tag_msg.detections[0].pose.pose.pose.position.x))
        if global_variables.tag_msg.detections[0].pose.pose.pose.position.y != []:
            y_value.set('{:2f}'.format(global_variables.tag_msg.detections[0].pose.pose.pose.position.y))
        if global_variables.tag_msg.detections[0].pose.pose.pose.position.z != []:
            z_value.set('{:2f}'.format(global_variables.tag_msg.detections[0].pose.pose.pose.position.z))
        if global_variables.tag_msg.detections[0].pose.pose.pose.orientation != []:
            orientation_value.set('{:2f}'.format(get_heading_from_quaternion(global_variables.tag_msg.detections[0].pose.pose.pose.orientation)))
        
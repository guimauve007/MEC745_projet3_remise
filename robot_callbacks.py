import global_variables

import rospy
from nav_msgs.msg import Odometry
from jackal_msgs.msg import Drive
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray


# Camera subscriber callback
def cam_callback(msg):
    global_variables.cam_msg = msg

# Odometry subscriber callback
def odometry_callback(msg):
    global_variables.odom_msg = msg
    #print(f"position x: {msg.pose.pose.position.x}, position y: {msg.pose.pose.position.y}, orientation: {msg.pose.pose.orientation}")

def publish_cmd_drive(cmd_drive_msg):
    global_variables.cmd_drive_pub.publish(cmd_drive_msg)

# Apriltag callback
def tag_callback(msg):
    global_variables.tag_msg  = msg

def subscribe(sim):
    global_variables.cam_msg = Image()
    global_variables.odom_msg = Odometry()
    global_variables.tag_msg = AprilTagDetectionArray()

    if sim:
        global_variables.cmd_drive_pub = rospy.Publisher('/mobile_manip/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
        global_variables.cam_sub = rospy.Subscriber('/mobile_manip/d435i/color/image_raw', Image, cam_callback)
    else:
        global_variables.cmd_drive_pub = rospy.Publisher('/mobile_manip/base/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
        global_variables.cam_sub = rospy.Subscriber('/mobile_manip/d435i/color/image_raw', Image, cam_callback)

    global_variables.odometry_sub = rospy.Subscriber('/odometry/filtered', Odometry, odometry_callback)
    global_variables.tag_sub = rospy.Subscriber('/mobile_manip/tag_detections', AprilTagDetectionArray, tag_callback)
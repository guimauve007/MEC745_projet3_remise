from jackal_msgs.msg import Drive
import robot_callbacks

# paramètres du robot
INTERWHEEL_DISTANCE = 0.434 # m
LEFT_WHEEL_RADIUS = 0.049 # m
RIGHT_WHEEL_RADIUS = 0.049 # m

# Vitesses pour le déplacement manuel
MANUAL_LINEAR_SPEED = 0.5 # m/s
MANUAL_ANGULAR_SPEED = 0.5 # rad/s

def move_robot(linear_speed, angular_speed):
    vel_left  = (linear_speed - angular_speed * INTERWHEEL_DISTANCE / 2.0) / LEFT_WHEEL_RADIUS
    vel_right = (linear_speed + angular_speed * INTERWHEEL_DISTANCE / 2.0) / RIGHT_WHEEL_RADIUS

    # Envoi des commandes au roues par topic ROS
    cmd_drive_msg = Drive()
    cmd_drive_msg.drivers[0] = vel_left
    cmd_drive_msg.drivers[1] = vel_right
    robot_callbacks.publish_cmd_drive(cmd_drive_msg)

def turn_robot_lef():
    move_robot(0, get_manual_angular_speed())

def turn_robot_right():
    move_robot(0, -get_manual_angular_speed())

def stop_robot():
    move_robot(0,0)

def get_manual_linear_speed():
    return MANUAL_LINEAR_SPEED

def get_manual_angular_speed():
    return MANUAL_LINEAR_SPEED

def activate_robot_arm():
    print("Robot arm activated!")
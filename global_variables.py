# Simulation or real status variable
global SIMULATION_STATUS

# Robot pathing variables
# Variables
global KP_ANGULAR
global KP_LINEAR
global TOLERANCE_ANGLE
global TOLERANCE_DISTANCE
global LINEAR_ANGULAR_SPEED

global x_destination
global y_destination
global moveToDestination
global createPath
global waypoint_index
global currentPath
global astarPlanner

# Callbacks
# Publishers
global cmd_drive_pub
global cam_sub
global odometry_sub
global tag_sub
# Messages
global cam_msg
global odom_msg

# Interface
global current_position_oval_id
global destination_position_oval_id
global marker_radius
global robot_display_status
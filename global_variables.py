global simu_status

# Robot pathing variables
# Variables
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
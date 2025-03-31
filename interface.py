import global_variables
import initialization
import robot_control
import robot_pathing
import robot_positioning
import tag_detection

import rospy

#path planning includes
from matplotlib import image as mpimg

## INTERFACE
from cv_bridge import CvBridge
import tkinter as tk
from PIL import Image, ImageTk

# Paramètres de la map
MAP_SCALING = 1/8

def interface_init():
    global_variables.current_position_oval_id     = 0
    global_variables.destination_position_oval_id = 0
    global_variables.marker_radius                = 2  # Radius of the marker in pixels
# *************************************************************************************************
# All initializations should be done here
rate = initialization.initialize_all()
interface_init()

# *************************************************************************************************

# Update display
# def updateTargetDestinationDisplay(x_clicked, y_clicked):
#     x_clicked_value.set('{:2f}'.format(x_clicked))
#     y_clicked_value.set('{:2f}'.format(y_clicked))  

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
    if (global_variables.cam_msg.height == 0):
        pass
    else:
        cv_image = bridge.imgmsg_to_cv2(global_variables.cam_msg, "rgb8")
        image = Image.fromarray(cv_image)
        image = resize_image(image, 400, 200) # Camera display size
        imageTk = ImageTk.PhotoImage(image=image)
        label.configure(image=imageTk)
        label.image = imageTk
        tag_detection.updateTagDetectionDisplay(x_value, y_value, z_value, orientation_value)
    root.after(10, update_plot)  

def update_robot_pathing():
    robot_pathing.process()
    root.after(250, update_robot_pathing)

def update_map():
    # Delete old position marker (ancient position)
    if global_variables.current_position_oval_id != 0:
        map.delete(global_variables.current_position_oval_id)

    x_position, y_position, theta = robot_positioning.get_robot_map_pixel_position()
    x_map_position = (x_position + robot_positioning.get_robot_map_offset_x()) * MAP_SCALING
    # We need to invert Y axis direction
    y_map_position = (-y_position + robot_positioning.get_robot_map_offset_y()) * MAP_SCALING

    global_variables.current_position_oval_id = map.create_oval(
        x_map_position - global_variables.marker_radius, y_map_position - global_variables.marker_radius,
        x_map_position + global_variables.marker_radius, y_map_position + global_variables.marker_radius,
        fill="green", outline="black"
    )
    map.pack()
    root.after(500, update_map)

def create_map_destination_marker(x, y):
    # Delete old destination marker (ancient position)
    if global_variables.destination_position_oval_id != 0:
        map.delete(global_variables.destination_position_oval_id)

    global_variables.destination_position_oval_id = map.create_oval(
        x - global_variables.marker_radius, y - global_variables.marker_radius,
        x + global_variables.marker_radius, y + global_variables.marker_radius,
        fill="yellow", outline="black"
    )
    map.pack()

# Canvas for map
def on_map_click(event):
    x_clicked, y_clicked = event.x, event.y
    print(f"Clicked at: ({x_clicked}, {y_clicked})")
    #updateTargetDestinationDisplay(x_clicked, y_clicked)
    create_map_destination_marker(x_clicked, y_clicked)
    x_click_scaled = x_clicked/MAP_SCALING
    y_click_scaled = y_clicked/MAP_SCALING
    robot_pathing.set_destination(x_click_scaled, y_click_scaled)
    robot_pathing.set_move_to_destination(True)
    robot_pathing.set_create_path(True)
# *************************************************************************************************
# Interface creation
root = tk.Tk()

# Main display
label = tk.Label(root)
label.pack()

# Load map image
map_image = Image.open("/home/jupyter-mecbotg11/Project5/Maps/a2230_map_closed.png")

map_width_scaled = int(map_image.width * MAP_SCALING)
map_height_scaled = int(map_image.height * MAP_SCALING)
map_image = map_image.resize((map_width_scaled, map_height_scaled))  # Resize the image
map_photo = ImageTk.PhotoImage(map_image)

map = tk.Canvas(root, width=map_width_scaled, height=map_height_scaled)
map.create_image(0, 0, anchor=tk.NW, image=map_photo)
# Draw a marker (a small red circle) at the origin coordinates
# Coordinates for the oval (circle) are defined by the top-left and bottom-right corners
x_map_position = (0 + robot_positioning.get_robot_map_offset_x()) * MAP_SCALING
y_map_position = (0 + robot_positioning.get_robot_map_offset_y()) * MAP_SCALING
origin_oval_id = map.create_oval(
    x_map_position - global_variables.marker_radius, y_map_position - global_variables.marker_radius,
    x_map_position + global_variables.marker_radius, y_map_position + global_variables.marker_radius,
    fill="red", outline="black"
)

map.bind("<Button-1>", on_map_click)
map.pack()
# *************************************************************************************************

# *************************************************************************************************
# Manual movement buttons
# Top manual button
manual_buttons_top_frame = tk.Frame(root)
manual_buttons_top_frame.pack()
btn_forward = tk.Button(manual_buttons_top_frame, text="Forward", command=lambda: robot_control.move_robot(robot_control.get_manual_linear_speed(), 0))
btn_forward.pack()

# Middle row buttons
manual_buttons_middle_frame = tk.Frame(root)
manual_buttons_middle_frame.pack()
btn_left = tk.Button(manual_buttons_middle_frame, text="Left", command=lambda: robot_control.turn_robot_lef())
btn_stop = tk.Button(manual_buttons_middle_frame, text="Stop", command=lambda: robot_control.stop_robot())
btn_right = tk.Button(manual_buttons_middle_frame, text="Right", command=lambda: robot_control.turn_robot_right())

btn_left.pack(side=tk.LEFT)
btn_stop.pack(side=tk.LEFT)
btn_right.pack(side=tk.LEFT)

# Bottom manual button
manual_buttons_bottom_frame = tk.Frame(root)
manual_buttons_bottom_frame.pack()
btn_backward = tk.Button(manual_buttons_bottom_frame, text="Backward", command=lambda: robot_control.move_robot(-robot_control.get_manual_linear_speed(), 0))
btn_backward.pack()
# *************************************************************************************************
# Specicif buttons
# Robot arm control and exit auto-pathing mode buttons
specific_buttons_frame = tk.Frame(root)
specific_buttons_frame.pack()
btn_exit_auto_pathing_mode = tk.Button(specific_buttons_frame, text="Exit auto-pathing", command=lambda: robot_pathing.force_exit_pathing_mode())
btn_activate_robot_arm = tk.Button(specific_buttons_frame, text="Activate robot arm", command=lambda: robot_control.activate_robot_arm())
btn_retract_robot_arm = tk.Button(specific_buttons_frame, text="Retract robot arm", command=lambda: robot_control.retract_robot_arm())

btn_exit_auto_pathing_mode.pack(side=tk.LEFT)
btn_activate_robot_arm.pack(side=tk.LEFT)
btn_retract_robot_arm.pack(side=tk.LEFT)
# *************************************************************************************************

# *************************************************************************************************
# Different text displays
# Create a main container frame
main_labels_frame = tk.Frame(root)
main_labels_frame.pack()

# Tag Detection Display
x_value = tk.DoubleVar()
y_value = tk.DoubleVar()
z_value = tk.DoubleVar()
orientation_value = tk.DoubleVar()

tag_frame = tk.Frame(main_labels_frame)

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

tag_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
# *************************************************************************************************
# Target destination display
# x_clicked_value = tk.DoubleVar()
# y_clicked_value = tk.DoubleVar()

# target_frame = tk.Frame(main_labels_frame)

# # Target Destination Label
# tk.Label(target_frame, text="Target Destination").pack()

# # Create a frame for each label-text pair
# x_clicked_frame = tk.Frame(target_frame)
# tk.Label(x_clicked_frame, text="x_clicked:").pack(side="left")
# tk.Label(x_clicked_frame, textvariable=x_clicked_value).pack(side="left")
# x_clicked_frame.pack()

# y_clicked_frame = tk.Frame(target_frame)
# tk.Label(y_clicked_frame, text="y_clicked:").pack(side="left")
# tk.Label(y_clicked_frame, textvariable=y_clicked_value).pack(side="left")
# y_clicked_frame.pack()

# target_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
# *************************************************************************************************
# Robot status display
global_variables.robot_display_status = tk.DoubleVar()

update_plot()
update_map()
update_robot_pathing()
root.mainloop()

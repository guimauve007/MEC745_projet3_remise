import global_variables
import initialization
import robot_callbacks
import robot_control
import robot_pathing
import robot_positioning
import tag_detection

from sensor_msgs.msg import Image

#path planning includes
from matplotlib import image as mpimg

## INTERFACE
from cv_bridge import CvBridge
import tkinter as tk
from PIL import Image, ImageTk

# *************************************************************************************************
# All initializations should be done here
rate = initialization.initialize_all()

# *************************************************************************************************

# Paramètres de la map
MAP_SCALING = 1/8

# Update display
def updateTargetDestinationDisplay(x_clicked, y_clicked):
    x_clicked_value.set('{:2f}'.format(x_clicked))
    y_clicked_value.set('{:2f}'.format(y_clicked))  

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
        image = resize_image(image, 960, 540)
        imageTk = ImageTk.PhotoImage(image=image)
        label.configure(image=imageTk)
        label.image = imageTk
        tag_detection.updateTagDetectionDisplay(x_value, y_value, z_value, orientation_value)
    root.after(10, update_plot)  

def update_robot_pathing():
    robot_pathing.process()
    root.after(10, update_robot_pathing)

root = tk.Tk()

# Main display
label = tk.Label(root)
label.pack()
buttons_frame = tk.Frame(root)
buttons_frame.pack()

btn_forward = tk.Button(buttons_frame, text="Forward", command=lambda: robot_control.move_robot(robot_control.get_manual_linear_speed(), 0))
btn_left = tk.Button(buttons_frame, text="Left", command=lambda: robot_control.move_robot(0, robot_control.get_manual_angular_speed()))
btn_backward = tk.Button(buttons_frame, text="Backward", command=lambda: robot_control.move_robot(-robot_control.get_manual_linear_speed(), 0))
btn_right = tk.Button(buttons_frame, text="Right", command=lambda: robot_control.move_robot(0, -robot_control.get_manual_angular_speed()))
btn_stop = tk.Button(buttons_frame, text="Stop", command=lambda: robot_control.move_robot(0,0))

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
    robot_pathing.set_destination(x_click_scaled, y_click_scaled)
    robot_pathing.set_move_to_destination(True)
    robot_pathing.set_create_path(True)

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
update_robot_pathing()
root.mainloop()

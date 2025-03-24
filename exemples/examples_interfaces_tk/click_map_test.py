import tkinter as tk
from tkinter import Canvas
from PIL import Image, ImageTk

scaling = 1/4
origin_x = 1444.7
origin_y = 1052.1

map_offset_x = origin_x * scaling
map_offset_y = origin_y * scaling

# Function to handle clicks on the image
def on_image_click(event):
    x, y = event.x, event.y  # Get click coordinates
    out = [x / scaling, y / scaling]  # Compute adjusted coordinates
    print(out)  # Print the coordinates to the console

# Create the Tkinter window
root = tk.Tk()
root.title("Image Click Example")

# Load the image
image_path = "/home/jupyter-mecbotg11/Project5/Maps/a2230_map_closed.png"
img = Image.open(image_path)
img = img.resize((int(img.width * scaling), int(img.height * scaling)))  # Resize the image
photo = ImageTk.PhotoImage(img)

# Create a Canvas widget to display the image
canvas = Canvas(root, width=img.width, height=img.height)
canvas.pack()

# Display the image on the Canvas
canvas.create_image(0, 0, anchor=tk.NW, image=photo)

# Draw a marker (a small red circle) at the origin coordinates
marker_radius = 2  # Radius of the marker in pixels
# Coordinates for the oval (circle) are defined by the top-left and bottom-right corners
canvas.create_oval(
    map_offset_x - marker_radius, map_offset_y - marker_radius,
    map_offset_x + marker_radius, map_offset_y + marker_radius,
    fill="red", outline="black"
)

# Bind the left mouse click event to the function
canvas.bind("<Button-1>", on_image_click)

# Start the Tkinter event loop
root.mainloop()

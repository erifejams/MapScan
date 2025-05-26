import cv2
import cv2.aruco as aruco
import numpy as np

# Marker and background settings
marker_id = 23
marker_size = 200  # size of the marker in pixels
canvas_size = 600  # total size of the white background (like a sheet of paper)

# Create the ArUco dictionary and marker
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
marker = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

# Create a white canvas
canvas = 255 * np.ones((canvas_size, canvas_size), dtype=np.uint8)

# Compute top-left corner for centering the marker
start_x = (canvas_size - marker_size) // 2
start_y = (canvas_size - marker_size) // 2

# Place the marker on the canvas
canvas[start_y:start_y+marker_size, start_x:start_x+marker_size] = marker

print("here")
# Save the image
cv2.imwrite(f"arucoImgs/aruco_marker_{marker_id}.png", canvas)

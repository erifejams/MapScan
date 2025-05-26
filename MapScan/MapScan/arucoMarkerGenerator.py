#generates different aruco marker of different sizes for each item in the dictionary
# code from : https://github.com/KhairulIzwan/ArUco-markers-with-OpenCV-and-Python/blob/main/Generating%20ArUco%20markers/opencv_generate_aruco.py

import os
import random
import imutils
import sys
import cv2
import cv2.aruco as aruco
import numpy as np
          
canvas_size = 600
marker_size = 300

# define names of each possible ArUco tag OpenCV supports
arucoDict = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}




for dict_name, typeDict in arucoDict.items():
	aruco_dict = aruco.getPredefinedDictionary(typeDict)

	for i in range(1): # generate for each in the dictionary
		id = random.randint(1, 20)
		img = aruco.generateImageMarker(aruco_dict, id, marker_size)

		# to make a white background for the aruco marker
		canvas = 255 * np.ones((canvas_size, canvas_size), dtype=np.uint8)

		start_x = (canvas_size - marker_size) // 2
		start_y = (canvas_size - marker_size) // 2

		# Place the marker on the canvas
		canvas[start_y:start_y+marker_size, start_x:start_x+marker_size] = img

		cv2.imwrite(f"arucoImgs/aruco_marker_{id}.png", canvas)

print("Markers generated.")

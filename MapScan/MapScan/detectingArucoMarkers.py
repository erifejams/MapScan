#got the video code from here https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

import argparse
import imutils
import cv2
import sys


def detectingMarkers():
	# define names of each possible ArUco tag OpenCV supports
	ARUCO_DICT = {
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
		"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
		"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
		"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
		"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
	}


	ap = argparse.ArgumentParser()
	ap.add_argument("-i", "--image", required=True,
		help="path to input image containing ArUCo tag")
	ap.add_argument("-t", "--type", type=str,
		default="DICT_ARUCO_ORIGINAL",
		help="type of ArUCo tag to detect")
	args = vars(ap.parse_args())



	if ARUCO_DICT.get(args["type"], None) is None:
		print("[INFO] ArUCo tag of '{}' is not supported".format(
			args["type"]))
		sys.exit(0)
	# load the ArUCo dictionary and grab the ArUCo parameters
	print("[INFO] detecting '{}' tags...".format(args["type"]))
	arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
	arucoParams = cv2.aruco.DetectorParameters_create()
	# initialize the video stream and allow the camera sensor to warm up
	print("[INFO] starting video stream...")
	#self.cv_image = cv2.VideoStream(src=0).start()
	#vs = cv2.VideoStream(src=0).start()
	# time.sleep(2.0)


	# loop over the frames from the video stream
	while True:
		# grab the frame from the threaded video stream and resize it
		# to have a maximum width of 1000 pixels
		#frame = vs.read()
		frame = imutils.resize(frame, width=1000)
		# detect ArUco markers in the input frame
		(corners, ids, rejected) = cv2.aruco.detectMarkers(frame,
			arucoDict, parameters=arucoParams)
		
		# self.cv_image = cv2.VideoStream(src=0).start()
        # #self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # cv2.imshow("Camera", self.cv_image)

        # cv2.waitKey(1)


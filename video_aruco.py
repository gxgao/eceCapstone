import cv2
import imutils
import pdb
import numpy as np
from imutils.video import VideoStream
import time
from cameraCalibration import loadCoefficients
import binID

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


# aruco class setup
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
parameters = cv2.aruco.DetectorParameters_create()

# detector = cv2.aruco.ArucoDetector(dictionary, parameters)

time.sleep(2.0)

vs = VideoStream(src=0).start()

cMtx, cDist = loadCoefficients()


# returns distance vector if (translation_vector, bin) was found else None 
# translation_vector = [x, y, z]
def fetch_bin_distance_vec(): 
	frame = vs.read()
	frame = imutils.resize(frame, width=1000)
	# detect ArUco markers in the input frame
    # markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)
	markerCorners, markerIds, rejected = cv2.aruco.detectMarkers(
	    frame, dictionary, parameters=parameters)

	if markerIds is not None and len(markerIds) != 0:
		# print(markerCorners)
		markerSizeInCM = 3.1
		rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, markerSizeInCM, cMtx, cDist)
		return (tvec[0][0], markerIds)

	return None 


def testf():
	# loop over the frames from the video stream
	while 1:
		# grab the frame from the threaded video stream and resize it
		# to have a maximum width of 1000 pixels
		print(fetch_bin_distance_vec()) 
			



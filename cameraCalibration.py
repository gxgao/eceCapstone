
# this is for 
# # code is mainly adapted from: https://medium.com/vacatronics/3-ways-to-calibrate-your-camera-using-opencv-and-python-395528a51615

# import numpy as np
# import cv2
# import cv2.aruco as aruco
# import pathlib
# import os 
# import pickle

# MARKER_LENGTH = 2.7
# SQUARE_LENGTH = 3.2

# # SOME UTIL FUNCTIONS 
# def save_coefficients(mtx, dist, path):
#     '''Save the camera matrix and the distortion coefficients to given path/file.'''
#     cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
#     cv_file.write('K', mtx)
#     cv_file.write('D', dist)
#     # note you *release* you don't close() a FileStorage object
#     cv_file.release()

# def load_coefficients(path):
#     '''Loads camera matrix and distortion coefficients.'''
#     # FILE_STORAGE_READ
#     cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

#     # note we also have to specify the type to retrieve other wise we only get a
#     # FileNode object back instead of a matrix
#     camera_matrix = cv_file.getNode('K').mat()
#     dist_matrix = cv_file.getNode('D').mat()

#     cv_file.release()
#     return [camera_matrix, dist_matrix]


# # generates the callibration image that we want
# def generateCharucoBoard(writeImg):
#     aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
#     board = aruco.CharucoBoard((5, 7), SQUARE_LENGTH, MARKER_LENGTH, aruco_dict)
#     if writeImg:
#         img = board.generateImage((2100, 2100))
#         cv2.imwrite("calibrationBoard.jpg", img)
#     return aruco_dict, board


# # does the callibration 
# def calibrate_charuco(dirpath, image_format, marker_length, square_length):
#     '''Apply camera calibration using aruco.
#     The dimensions are in cm.
#     '''
#     #aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
#     aruco_dict, board  = generateCharucoBoard(False)# aruco.CharucoBoard(5, 7, square_length, marker_length, aruco_dict)
#     #arucoParams = aruco.DetectorParameters_create()
#     parameters =  cv2.aruco.CharucoParameters()
#     CharucoDetector = cv2.aruco.CharucoDetector(board)

#     # detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

#     counter, corners_list, id_list = [], [], []
#     img_dir = pathlib.Path(dirpath)
#     first = 0

#     # Find the ArUco markers inside each image
#     for img in img_dir.glob(f'*{image_format}'):
#         print(f'using image {img}')
#         image = cv2.imread(str(img))
#         img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#         # corners, ids, rejected = CharucoDetector.detectBoard(
#         #     img_gray
#         # )
#         corners, ids = np.array([]), np.array([])
#         # resp, charuco_corners, charuco_ids = CharucoDetector.interpolateCornersCharuco(
#         #     markerCorners=corners,
#         #     markerIds=ids,
#         #     image=img_gray,
#         #     board=board
#         # )
#         resp, charuco_corners, charuco_ids, marker_ids = CharucoDetector.detectBoard(
#             img_gray
#         )
#         # If a Charuco board was found, let's collect image/corner points
#         # Requiring at least 20 squares
#         if len(resp) > 20:
#             # Add these corners and ids to our calibration arrays
#             corners_list.append(charuco_corners)
#             id_list.append(charuco_ids)

#     # Actual calibration
#     ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraCharucoExtended(
#         corners_list, 
#         id_list, 
#         board, 
#         img_gray.shape, 
#         None, 
#         None)
    
#     return [ret, mtx, dist, rvecs, tvecs]


# # from utils import load_coefficients, save_coefficients

# # Parameters
# IMAGES_DIR = 'calibrationImages'
# CAM_COEFF_DIR = "cameraCoefficients"
# CAM_COEFF_NAME = "calibration_charuco.yml"
# IMAGES_FORMAT = 'jpg'
# # Dimensions in cm
# MARKER_LENGTH = 3.1
# SQUARE_LENGTH = 3.7

# def generateCallibration():
#     if (len(os.listdir("cameraCoefficients")) > 0):
#         return 
#     # Calibrate 
#     ret, mtx, dist, rvecs, tvecs = calibrate_charuco(
#         IMAGES_DIR, 
#         IMAGES_FORMAT,
#         MARKER_LENGTH,
#         SQUARE_LENGTH
#     )

#     # f = open(IMAGES_DIR + "\\" + CAM_COEFF_NAME, 'wb')
#     # pickle.dump((mtx, dist, rvecs, tvecs), f)
#     # f.close()
#     # Save coefficients into a file
#     save_coefficients(mtx, dist, IMAGES_DIR + "\\" + CAM_COEFF_NAME) #IMAGES_DIR + "\\" + "calibration_charuco.yml")

# # Load coefficients pickling not sure if needed 
# def loadCoefficients():
    
#     mtx, dist = load_coefficients(IMAGES_DIR + "\\" + CAM_COEFF_NAME)

#     return mtx, dist

# def testUndistort():
#     original = cv2.imread(IMAGES_DIR + "\\undistortTest.jpg")
#     mtx, dist = loadCoefficients() 
#     dst = cv2.undistort(original, mtx, dist, None, mtx)
#     cv2.imwrite('undist_charuco.jpg', dst)


# if __name__ == "__main__":
#     #generateCharucoBoard(True) 

#     generateCallibration() 
#     testUndistort() 

# """
# Aruco camera calibration
# """

# # Import required packages:
# import time
# import cv2
# import numpy as np
# import pickle

# # Create dictionary and board object:
# dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_250)
# board = cv2.aruco.CharucoBoard_create(3, 3, .025, .0125, dictionary)

# # Create board image to be used in the calibration process:
# image_board = board.draw((200 * 3, 200 * 3))

# # Write calibration board image:
# cv2.imwrite('charuco.png', image_board)

# # Create VideoCapture object:
# cap = cv2.VideoCapture(0)

# all_corners = []
# all_ids = []
# counter = 0
# for i in range(300):

#     # Read frame from the webcam:
#     ret, frame = cap.read()

#     # Convert to grayscale:
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Detect markers:
#     res = cv2.aruco.detectMarkers(gray, dictionary)

#     if len(res[0]) > 0:
#         res2 = cv2.aruco.interpolateCornersCharuco(res[0], res[1], gray, board)
#         if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3 and counter % 3 == 0:
#             all_corners.append(res2[1])
#             all_ids.append(res2[2])

#         cv2.aruco.drawDetectedMarkers(gray, res[0], res[1])

#     cv2.imshow('frame', gray)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
#     counter += 1

# # Calibration can fail for many reasons:
# try:
#     cal = cv2.aruco.calibrateCameraCharuco(all_corners, all_ids, board, gray.shape, None, None)
# except:
#     cap.release()
#     print("Calibration could not be done ...")

# # Get the calibration result:
# retval, cameraMatrix, distCoeffs, rvecs, tvecs = cal
# # Save the camera parameters:
# f = open('calibration2.pckl', 'wb')
# pickle.dump((cameraMatrix, distCoeffs), f)
# f.close()

# # Release everything:
# cap.release()
# cv2.destroyAllWindows()

# code is mainly adapted from: https://medium.com/vacatronics/3-ways-to-calibrate-your-camera-using-opencv-and-python-395528a51615
# this code is for 4.5.5.64 
import numpy as np
import cv2
import cv2.aruco as aruco
import pathlib
import os 
import pickle

MARKER_LENGTH = 2.7
SQUARE_LENGTH = 3.2

# SOME UTIL FUNCTIONS 
def save_coefficients(mtx, dist, path):
    '''Save the camera matrix and the distortion coefficients to given path/file.'''
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write('K', mtx)
    cv_file.write('D', dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()

def load_coefficients(path):
    '''Loads camera matrix and distortion coefficients.'''
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode('K').mat()
    dist_matrix = cv_file.getNode('D').mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]


# generates the callibration image that we want
def generateCharucoBoard(writeImg):
    aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    board = aruco.CharucoBoard_create(5, 7, SQUARE_LENGTH, MARKER_LENGTH, aruco_dict)
    if writeImg:
        img = board.generateImage((2100, 2100))
        cv2.imwrite("calibrationBoard.jpg", img)
    return aruco_dict, board


# does the callibration 

def calibrate_charuco(dirpath, image_format, marker_length, square_length):
    '''Apply camera calibration using aruco.
    The dimensions are in cm.
    '''
    #aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
    aruco_dict, board = generateCharucoBoard(False) #aruco.CharucoBoard_create(5, 7, square_length, marker_length, aruco_dict)
    arucoParams = aruco.DetectorParameters_create()

    counter, corners_list, id_list = [], [], []
    img_dir = pathlib.Path(dirpath)
    first = 0
    # Find the ArUco markers inside each image
    for img in img_dir.glob(f'*{image_format}'):
        print(f'using image {img}')
        image = cv2.imread(str(img))
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(
            img_gray, 
            aruco_dict, 
            parameters=arucoParams
        )

        resp, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            markerCorners=corners,
            markerIds=ids,
            image=img_gray,
            board=board
        )
        # If a Charuco board was found, let's collect image/corner points
        # Requiring at least 20 squares
        if resp > 20:
            # Add these corners and ids to our calibration arrays
            corners_list.append(charuco_corners)
            id_list.append(charuco_ids)

    # Actual calibration
    ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraCharuco(
        charucoCorners=corners_list, 
        charucoIds=id_list, 
        board=board, 
        imageSize=img_gray.shape, 
        cameraMatrix=None, 
        distCoeffs=None)
    
    return [ret, mtx, dist, rvecs, tvecs]


# from utils import load_coefficients, save_coefficients

# Parameters
IMAGES_DIR = 'calibrationImages'
CAM_COEFF_DIR = "cameraCoefficients"
CAM_COEFF_NAME = "calibration_charuco.yml"
IMAGES_FORMAT = 'jpg'
# Dimensions in cm
MARKER_LENGTH = 3.1
SQUARE_LENGTH = 3.7

def generateCallibration():
    if (len(os.listdir("cameraCoefficients")) > 0):
        return 
    # Calibrate 
    ret, mtx, dist, rvecs, tvecs = calibrate_charuco(
        IMAGES_DIR, 
        IMAGES_FORMAT,
        MARKER_LENGTH,
        SQUARE_LENGTH
    )

    # f = open(IMAGES_DIR + "\\" + CAM_COEFF_NAME, 'wb')
    # pickle.dump((mtx, dist, rvecs, tvecs), f)
    # f.close()
    # Save coefficients into a file
    save_coefficients(mtx, dist, IMAGES_DIR + "/" + CAM_COEFF_NAME) #IMAGES_DIR + "\\" + "calibration_charuco.yml")

# Load coefficients pickling not sure if needed 
def loadCoefficients():
    
    mtx, dist = load_coefficients(IMAGES_DIR + "/" + CAM_COEFF_NAME)

    return mtx, dist

def testUndistort():
    original = cv2.imread(IMAGES_DIR + "\\undistortTest.jpg")
    mtx, dist = loadCoefficients() 
    dst = cv2.undistort(original, mtx, dist, None, mtx)
    cv2.imwrite('undist_charuco.jpg', dst)


if __name__ == "__main__":
    #generateCharucoBoard(True) 

    generateCallibration() 
    testUndistort() 

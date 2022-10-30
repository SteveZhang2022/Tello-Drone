"""
    Here, the main route for the general movement for this
    drone will be done here and all needed modules will 
    be imported here. 

"""
from time import sleep
from djitellopy import tello
from modules import keyPressModule as kpm
from time import sleep
from getBattery import baReminder
import cv2 as cv
from cv2 import aruco
import numpy as np
import math

# wid1 = input("Enter wanted id1:")
# wid2 = input("Enter wanted id2:")
# wid3 = input("Enter wanted id3:")

wid1 = 0
wid2 = 15
wid3 = 26


cidlist = []
cidxlist = []
count = 0

kpm.init()
drone = tello.Tello()
drone.connect()
baReminder()
drone.streamoff()
drone.streamon()
marker_length = 10  # 10 cm
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

camera_matrix = np.array([[2.13243514e+03, 0.00000000e+00, 1.33300826e+03],
                          [0.00000000e+00, 2.15409995e+03, 4.95966668e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

distortion_coefficients = np.array([[0.20388981, -0.66295284, -0.0584427, 0.00772092, 1.17854164]])

axis = np.array([[marker_length, 0, 0], [0, marker_length, 0], [0, 0, marker_length * -1]]).reshape(-1, 3)
#cap = cv.VideoCapture(0)

def getKeyBoardInput():
    # lr is left & right
    # fb is front & back
    # ud is up & down
    # yv is yaw & velocity
    lr, fb, ud,  yv = 0, 0, 0, 0
    speed = 50

    if kpm.getKey("LEFT"): lr  = -speed
    elif kpm.getKey("RIGHT"): lr  = speed

    if kpm.getKey("UP"): fb  = speed

    elif kpm.getKey("DOWN"): fb  = -speed

    if kpm.getKey("w"): ud  = speed
    elif kpm.getKey("s"): ud  = -speed

    if kpm.getKey("a"): yv  = speed
    elif kpm.getKey("d"): yv  = -speed

    if kpm.getKey("q"): drone.land()
    if kpm.getKey("t"): drone.takeoff()
    return [lr, fb, ud, yv]

while True:
    vals = getKeyBoardInput()
    drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])

#    frame = cap.read()
    frame = drone.get_frame_read().frame
    frame = cv.resize(frame, (640, 480 ))

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    #aruco.drawDetectedMarkers(frame, marker_corners, marker_IDs)
    if marker_IDs is not None:
        img_aruco = aruco.drawDetectedMarkers(frame, marker_corners, marker_IDs, (0, 255, 0))

        # tvec is the center of the marker in the camera's world
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(marker_corners, marker_length, camera_matrix,
                                                        distortion_coefficients)

        #print("tvec", tvec)
        #print("rvec", rvec)

        # In case there are multiple markers
        for i in range(marker_IDs.size):
            img_aruco = cv.drawFrameAxes(img_aruco, camera_matrix, distortion_coefficients, rvec[i], tvec[i],
                                         marker_length, 3)
#           img_aruco = aruco.drawAxis(img_aruco, camera_matrix, distortion_coefficients, rvec[i], tvec[i],
#                                       marker_length)

#        if rvec.size == 3:
#            imgpts, _ = cv.projectPoints(axis, rvec, tvec, camera_matrix, distortion_coefficients)

#            p1 = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))

#            if p1 is not None:
#                cv.circle(img_aruco, p1, 5, (0, 0, 255), -1)

#            p2 = (int(imgpts[1][0][0]), int(imgpts[1][0][1]))

#            if p2 is not None:
#                cv.circle(img_aruco, p2, 5, (0, 255, 0), -1)

#            p3 = (int(imgpts[2][0][0]), int(imgpts[2][0][1]))

#            if p3 is not None:
#                cv.circle(img_aruco, p3, 5, (255, 0, 0), -1)

#            # Plot a point at the center
#            cv.circle(img_aruco, (480, 320), 5, (0, 255, 0), -1)

            tvec_x = tvec[0][0][0]
            tvec_y = tvec[0][0][1]
            tvec_z = tvec[0][0][2]

            # Distance from camera is the magnitude of tvec
            distance = math.sqrt(tvec_x * tvec_x + tvec_y * tvec_y + tvec_z * tvec_z)

            # Let's focus on keeping the marker centered on the x axis (roll left/right)
            # This means we'll consider y and z constant for this demonstration

            # Calculate angle of vectors
            array = np.array([tvec_x, tvec_y, tvec_z])
            array_mag = np.linalg.norm(array)

            # Vector to center of screen
            array2 = np.array([0, 0, tvec_z])
            array2_mag = np.linalg.norm(array2)

            dot = np.dot(array, array2)

            # Solve for angle
            cos = np.arccos(dot / (array_mag * array2_mag))

            degrees = np.degrees(cos)

    #        for ids, corners in zip(marker_IDs, marker_corners):
#            cv.polylines(
#               frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
#            )
        if len(marker_IDs) == 18 and count == 0:
            count = 1
            list18 = zip(marker_IDs, marker_corners)
            for ids1, marker_corners1 in list18:
                # print(ids1, " ", corners1.astype(np.int32))
                if ids1[0] == wid1 or ids1[0] == wid2 or ids1[0] == wid3:
                    cidlist.append(ids1[0])
                    cidxlist.append(marker_corners1[0][0][0])
                    cidxlist.sort()

#           break

    print("***************one list printed****************")

print(cidlist)
print(cidxlist)
    cv.imshow("frame", frame)
    cv.waitKey(1)

#drone.land()
cv.destroyAllWindows()






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
targetList = [0, 15, 26]

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

marker_range = [6200, 6800]
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


def trackARuCO(tvec1, area):
    fb =0
    x = tvec1[0][0][0]
    y = tvec1[0][0][1]
    z = tvec1[0][0][2]

#    if area > marker_range[0] and area < marker_range[1]:
#        fb = 0
#    elif area > marker_range[1]:
#        fb = -20
#    elif area < marker_range[0] and area !=0:
#        fb = 20


    if z < 200:
        fb = -20
    elif z > 200:
        fb = 20
    else:
        fb = 0

    if x > 100:
        lr = 20
    elif x < -100:
        lr = -20
    else:
        lr = 0

    if y > 100:
        ud = 20
    elif y < -100:
        ud = -20
    else:
        ud = 0

drone.takeoff()
drone.move_up(50)

while True:
    vals = getKeyBoardInput()
    drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])

#    frame = cap.read()
    frame = drone.get_frame_read().frame
    frame = cv.resize(frame, (360, 240))

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_IDs is not None:
        img_aruco = aruco.drawDetectedMarkers(frame, marker_corners, marker_IDs, (0, 255, 0))

        if len(marker_IDs) == 9:
            for ids1, marker_corners1 in zip(marker_IDs, marker_corners):
                print(ids1, " ", marker_corners1.astype(np.int32))
                if ids1[0] == wid1 or ids1[0] == wid2 or ids1[0] == wid3:
                    cidlist.append(ids1[0])
                    cidxlist.append(marker_corners1[0])
            print("cidlist:", cidlist)
            print("cidxlist: ", cidxlist)
        # tvec is the center of the marker in the camera's world
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(cidxlist, marker_length, camera_matrix,
                                                        distortion_coefficients)

            print("tvec", tvec)
            print("rvec", rvec)

            # In case there are multiple markers
            for i in range(len(cidlist)):
                img_aruco = cv.drawFrameAxes(img_aruco, camera_matrix, distortion_coefficients, rvec[i], tvec[i],
                                         marker_length, 3)

            tvec_x = tvec[0][0][0]
            tvec_y = tvec[0][0][1]
            drone.move_forward((int(tvec[0][0][2]/10)))
            if (int(tvec_x/16) > 20):
                drone.move_right((int(tvec_x/16)))
            elif (int(tvec_x/(-16) > 20)):
                drone.move_left(int(tvec_x/(-16)))
            if (int(tvec_y/16)) > 20:
                drone.move_up(int(tvec_y/16))
            elif (int(tvec_y/(-16)) > 20):
                drone.move_down(int(tvec_y/(-16)))
        elif len(marker_IDs) == 1:
            print("only see one marker now")
            for ids1, marker_corners1 in zip(marker_IDs, marker_corners):
                print(ids1, " ", marker_corners1.astype(np.int32))
                if ids1[0] in targetList:
                    rvec1, tvec1, _ = aruco.estimatePoseSingleMarkers(marker_corners1, marker_length, camera_matrix,
                                                                      distortion_coefficients)
                    print("tvec1:", tvec1)
                    print("ids1:", ids1)
                    area = (marker_corners1[0][1]-marker_corners1[0][0])*(marker_corners1[0][2]-marker_corners1[0][1])
                    trackARuCO(tvec1, area)


    cv.imshow("frame", frame)
    cv.waitKey(1)


#drone.land()
cv.destroyAllWindows()






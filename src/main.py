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
import itertools
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
#cap = cv.VideoCapture(1)
marker_length = 10  # 10 cm
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

camera_matrix = np.array([[2.13243514e+03, 0.00000000e+00, 1.33300826e+03],
                          [0.00000000e+00, 2.15409995e+03, 4.95966668e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

distortion_coefficients = np.array([[0.20388981, -0.66295284, -0.0584427, 0.00772092, 1.17854164]])

axis = np.array([[marker_length, 0, 0], [0, marker_length, 0], [0, 0, marker_length * -1]]).reshape(-1, 3)


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


def trackARuCO(x, y, z):

    print("x, y, z: ", x, y, z)
#    if area > marker_range[0] and area < marker_range[1]:
#        fb = 0
#    elif area > marker_range[1]:
#        fb = -20
#    elif area < marker_range[0] and area !=0:
#        fb = 20


    if z < 600 and z > 0:
        fb = -10
    elif z > 600:
        fb = 10
    else:
        fb = 0

    if x > 450:
        lr = 10
    elif x < -450:
        lr = -10
    else:
        lr = 0

    if y > 200:
        ud = 10
    elif y < -200:
        ud = -10
    else:
        ud = 0


    drone.send_rc_control(lr, fb, ud, 0)
    print("send_rc_control ", lr," ",fb," ",ud, " 0")
    sleep(1.5)
    drone.send_rc_control(0,0,0,0)
    sleep(0.5)

def convert3to2(inputList):
    outputList = list(itertools.chain.from_iterable(inputList))
#    outputList = [sub[0] for sub in inputList]
    return outputList

def Sort(inputList):
    inputList.sort(key = lambda x: x[0])
    return inputList

drone.takeoff()
drone.move_up(40)

while True:
    tvec_x = 0
    tvec_y = 0
    tvec_z = 0
    cidlist = []
    cidxlist = []
    tvec_2 = []
    tvec = []
    rvec = []
    tvec_sorted = []

    vals = getKeyBoardInput()
    drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])

    frame = drone.get_frame_read().frame
    frame = cv.resize(frame, (360, 240))

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_IDs is not None:
        img_aruco = aruco.drawDetectedMarkers(frame, marker_corners, marker_IDs, (0, 255, 0))


        for ids1, marker_corners1 in zip(marker_IDs, marker_corners):
            print(ids1, " ", marker_corners1.astype(np.int32))
            if ids1[0] in targetList:
                cidlist.append(ids1[0])
                cidxlist.append(marker_corners1[0])
        print("cidlist:", cidlist)
        print("cidxlist: ", cidxlist)
                # tvec is the center of the marker in the camera's world
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(cidxlist, marker_length, camera_matrix,
                                                        distortion_coefficients)

        print("tvec ", tvec)
    #   print("rvec ", rvec)

            # In case there are multiple markers
        for i in range(len(cidlist)):
            img_aruco = cv.drawFrameAxes(img_aruco, camera_matrix, distortion_coefficients, rvec[i], tvec[i],
                                         marker_length, 3)

        if tvec is not None:
            tvec_2 = convert3to2(tvec)
            print("tvec_2: ", tvec_2)
            tvec_sorted = Sort(tvec_2)
            print("sorted tvec_2 ", tvec_sorted)
            tvec_x = tvec_sorted[0][0]
            tvec_y = tvec_sorted[0][1]
            tvec_z = tvec_sorted[0][2]
        trackARuCO(tvec_x, tvec_y, tvec_z)


    cv.imshow("frame", frame)
    cv.waitKey(1)


#drone.land()
cv.destroyAllWindows()






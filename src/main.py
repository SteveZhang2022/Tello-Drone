"""
    Here, the main route for the general movement for this
    drone will be done here and all needed modules will 
    be imported here. 

"""
from time import sleep
from djitellopy import tello
from getBattery import baReminder
import cv2 as cv
from cv2 import aruco

def main():
    drone = tello.Tello()
    drone.connect()

    drone.takeoff()
    drone.move_up(10)
    sleep(60)

    marker_dict = aruco.Dictionary_get(aruco.DICT_4x4_50)

    param_markers = aruco.DetectorParameters_create()

    cam = cv.VideoCapture(0)

    while True:
        ret, frame = cam.read()
        if not ret:
            break
        grey_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
        for ids, corners in zip(marker_IDs, marker_corners):
            print(ids, " ", corners)

        cv.imshow("frame", frame)
        key = cv.waitKey(1)
        if key == ord('q'):
            break
    cam.release()
    cv.destroyAllWindows()

    drone.land()

    # Function checks the battery status and prints out the exact percentage
    baReminder()

main()
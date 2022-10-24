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
    drone.move_up(60)
#    sleep(60)

    drone.streamoff()
    drone.streamon()
    marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

    param_markers = aruco.DetectorParameters_create()

    cam = cv.VideoCapture(1)



    while True:
        frame_read = drone.get_frame_read()
        frame = frame_read.frame
        frame = cv.resize(frame, (500, 350))
        cv.imshow("frame", frame)

        grey_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            grey_frame, marker_dict, parameters=param_markers
        )
        for ids, corners in zip(marker_IDs, marker_corners):
            print(ids, " ", corners)


        key = cv.waitKey(1)
        if key == ord('q'):
            break
#    cam.release()
    cv.destroyAllWindows()

    drone.land()

    # Function checks the battery status and prints out the exact percentage
    baReminder()

main()
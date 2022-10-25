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
import pygame

def init():
    pygame.init()
    win = pygame.display.set_mode((400,400))

def getKey(keyName):
    ans = False
    for eve in pygame.event.get(): pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame, 'K_{}'.format(keyName))
    if keyInput[myKey]:
        ans = True
    pygame.display.update()
    return ans

def main():
    init()
    drone = tello.Tello()
    drone.connect()
    print(drone.get_battery())

    drone.takeoff()
    if getKey("UP"):
        print("UP key pressed")
        drone.move_up(60)
#    sleep(60)

    drone.streamoff()
    drone.streamon()
    marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

    param_markers = aruco.DetectorParameters_create()

 #   cam = cv.VideoCapture(1)



    while True:
        frame = drone.get_frame_read().frame

        frame = cv.resize(frame, (360, 240))
        cv.imshow("frame", frame)

        grey_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            grey_frame, marker_dict, parameters=param_markers
        )
        for ids, corners in zip(marker_IDs, marker_corners):
            print(ids, " ", corners)


        cv.waitKey(1)
        if getKey("s"):
            print("s key pressed")
            break
#    cam.release()
    cv.destroyAllWindows()

    drone.land()

    # Function checks the battery status and prints out the exact percentage
    baReminder()

main()
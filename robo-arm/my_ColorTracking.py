#!/usr/bin/python3
# coding=utf8
import sys
from ColorTrackingClass import Perception
sys.path.append('/home/pi/ArmPi/')
import cv2
import Camera

if __name__ == '__main__':
    prcpt = Perception(_target_colors = ('red'))
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = prcpt.run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()

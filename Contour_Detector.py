#Imports
import sys
import imp
import cv2
import cscore as cs
import numpy as np
import math
from networktables import NetworkTables
from networktables.util import ntproperty
from time import sleep
from sys import exit


#Define contour detector function
def detect_contours():

    #Set known object values
    #FOV_angle_in_degrees = 23.55
    FOV_angle_in_degrees = 29.5
    Width_in_inches = 3.875
    
    #Set object test values
    minarea = 500

    #Set image properties
    imgwidth = 320
    imgheight = 240
    
    #Define video capture from webcam
    cam = cv2.VideoCapture(0)
    
    #set video capture properties
    ret = cam.set(3,imgwidth)
    ret = cam.set(4,imgheight)
    ret = cam.set(10,0.5)

    #set video codec and create VideoWriter
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #imgout = cv2.VideoWriter('/home/pi/Videos/robotcam.avi',fourcc,20.0,(320,240))

    #start capturing and processing video frames
    while (True):

        #Grab a frame
        ret_val, img = cam.read()

        #Blur image to remove noise
        blur = cv2.GaussianBlur(img.copy(),(5,5),0)
        
        #Convert from BGR to HSV colorspace
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #Define range of target color in HSV
        #targetMin = (97, 66, 94)
        #targetMax = (125, 228, 236)
        targetMin = (43, 97, 118)
        targetMax = (163, 239, 243)

        #Set pixels to white if in target HSV range, else set to black
        mask = cv2.inRange(hsv, targetMin, targetMax)

        #Find contours in mask
        image, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        #Find bounding rectangles of largest objects
        item = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area > minarea:
                item = item + 1
                x,y,w,h = cv2.boundingRect(c)
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
                dist = (Width_in_inches * imgwidth) / (2 * w * math.tan(math.radians(FOV_angle_in_degrees)))
                print('Width of object ', item, ' is: ', w)
                print('Distance to object ', item, ' is: ', dist)
                #print('The area is: ', area)
                #print('The aspect ratio is: ', float(w)/h)

        #Write frame to video file
        #imgout.write(img)

        #Show contours and mask
        cv2.imshow('My Webcam', img)
        #cv2.imshow('Mask', mask)

        #check for ESC key press
        if cv2.waitKey(1) == 27:
            break

    #release webcam (stop video feed)
    cam.release()

    #close video file
    #imgout.release()

    #close all open windows
    cv2.destroyAllWindows()



#define main function
def main():
    detect_contours()

if __name__ == '__main__':
    main()

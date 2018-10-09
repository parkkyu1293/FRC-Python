#!/usr/bin/env python3

#System imports
import sys
import os
import imp

#Setup paths
sys.path.append('/home/pi/.local/lib/python3.5/site-packages')
sys.path.append('/usr/local/lib/vmxpi/')

#Module imports
import cv2 as cv
import cscore as cs
import numpy as np
import math
import datetime
import time

from networktables import NetworkTables
from networktables.util import ntproperty
from time import sleep
from sys import exit


#Set image properties
imgwidth = 320
imgheight = 240
frames_per_sec = 15

#Set program flags
writeVideo = True

#Define contour detector function
def detect_contours(img_raw):

    #Set global variables
    global imgwidth
    
    #Set known object values
    #FOV_angle_in_degrees = 23.55
    FOVAngleInDegrees = 29.5
    WidthInInches = 3.875
    
    #Set object test values
    minarea = 2000
    
    #Blur image to remove noise
    blur = cv.GaussianBlur(img_raw.copy(),(5,5),0)
        
    #Convert from BGR to HSV colorspace
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

    #Define range of target color in HSV
    #targetMin = (97, 66, 94)
    #targetMax = (125, 228, 236)
    targetMin = (43, 97, 118)
    targetMax = (163, 239, 243)

    #Set pixels to white if in target HSV range, else set to black
    mask = cv.inRange(hsv, targetMin, targetMax)

    #Find contours in mask
    image, contours, hierarchy = cv.findContours(mask,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)

    #Find bounding rectangles of largest objects
    dist = 0
    angle = 0
    img_contours = cv.rectangle(img_raw,(0,0),(0,0),(0,0,255),2)
    for c in contours:

        #Get area of contour
        area = cv.contourArea(c)

        #Check area before further processing
        if area > minarea:

            #Get bounding rectangle and draw on image
            x,y,w,h = cv.boundingRect(c)
            img_contours = cv.rectangle(img_raw,(x,y),(x+w,y+h),(0,0,255),2)

            #Calculate distance to object
            dist = (WidthInInches * imgwidth) / (2 * w * math.tan(math.radians(FOVAngleInDegrees)))

            #Calculate angle to object
            inchesPerPixel = WidthInInches / w
            centerOffsetInInches = ((x + w / 2) - (imgwidth / 2)) * inchesPerPixel
            angle = math.degrees(math.atan(centerOffsetInInches / dist))
            

    #return results
    return img_contours, dist, angle


#define main processing function
def mainloop():

    #Open a log file
    log_file = open('/home/pi/Team4121/Run_Log.txt', 'w')
    log_file.write('Run started on %s.\n' % datetime.datetime.now())
    log_file.write('')

    #Load VMX module
    vmxpi = imp.load_source('vmxpi_hal_python', '/usr/local/lib/vmxpi/vmxpi_hal_python.py')
    vmx = vmxpi.VMXPi(False,50)
    if vmx.IsOpen() is False:
        log_file.write('Error:  Unable to open VMX Client.\n')
        log_file.write('\n')
        log_file.write('        - Is pigpio (or the system resources it requires) in use by another process?\n')
        log_file.write('        - Does this application have root privileges?')
        log_file.close()
        sys.exit(0)
        
    #Open connection to USB camera (video device 0)
    camera = cs.UsbCamera("usbcam", 0)
    camera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, imgwidth, imgheight, frames_per_sec)

    #Start raw Video Streaming Server
    mjpegServer = cs.MjpegServer("httpserver", 8081)
    mjpegServer.setSource(camera)

    #Define video sink
    cvsink = cs.CvSink("cvsink")
    cvsink.setSource(camera)

    #Define video source
    cvsource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, width, height, frames_per_sec)

    #Start processed Video Streaming Server
    cvMjpegServer = cs.MjpegServer("cvhttpserver", 8082)
    cvMjpegServer.setSource(cvsource)

    #Create blank image
    img = np.zeros(shape=(imgheight, imgwidth, 3), dtype=np.uint8)

    #Set video codec and create VideoWriter
    fourcc = cv.VideoWriter_fourcc(*'XVID')
    currentTime = time.localtime(time.time())
    timeString = str(currentTime.tm_year) + str(currentTime.tm_mon) + str(currentTime.tm_mday) + str(currentTime.tm_hour) + str(currentTime.tm_min)
    videoFilename = '/data/robotcam-' + timeString + '.avi'
    imgout = cv.VideoWriter(videoFilename,fourcc,20.0,(320,240))

    #Connect NetworkTables
    NetworkTables.initialize(server="192.168.0.113")

    #Initialize NetworkTable values
    ntNavxSensorTimestamp = ntproperty('/vmx/navxSensorTimestamp', 0)
    ntNavxAngle = ntproperty('/vmx/navxAngle', 0)
    ntNavxYVel = ntproperty('/vmx/navxYVel', 0)
    ntNavxXVel = ntproperty('/vmx/navxXVel', 0)
    ntNavxYDisp = ntproperty('/vmx/navxYDisp', 0)
    ntNavxXDisp = ntproperty('/vmx/navxXDisp', 0)
    ntCubeAngle = ntproperty('/vision/cubeAngle', 0)
    ntCubeDist = ntproperty('/vision/cubeDist', 0)
    ntCubeHeight = ntproperty('/vision/cubeHeight', 0)
    
    #Reset yaw gyro
    if not vmx.getAHRS().IsCalibrating:
        vmx.getAHRS().ZeroYaw()

    #Reset displacement
    vmx.getAHRS().ResetDisplacement()

    #Start main processing loop
    while (True):

        #Grab a frame from video feed
        video_timestamp, img = cvsink.grabFrame(img)

        #Check for frame error
        if video_timestamp == 0:
            log_file.write('Video error: \n')
            log_file.write(cvsink.getError())
            log_file.write('\n')
            sleep (float(frames_per_sec * 2) / 1000.0)
            continue

        #Find contours in image
        img_contours, cubeDistance, cubeAngle = detect_contours(img)

        #Put contour info on image
        distanceString = 'Distance: %d' % cubeDistance
        cv.putText(img_contours, distanceString, (10,15), cv.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv.LINE_AA)
        angleString = 'Offset angle: %d' % cubeAngle
        cv.putText(img_contours, angleString, (10,30), cv.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv.LINE_AA)

        #Put IMU info on image
        yawString = 'Yaw:  %.2f' % vmx.getAHRS().GetYaw()
        cv.putText (img, yawString, (30,50), cv.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv.LINE_AA)
        angleString = 'Angle: %.2f' % vmx.getAHRS().GetAngle()
        cv.putText (img, angleString, (30,70), cv.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv.LINE_AA)
        yVelocityString = 'Y Vel:  %.2f' % vmx.getAHRS().GetVelocityY()
        cv.putText (img, yVelocityString, (30,90), cv.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv.LINE_AA)
        xVelocityString = 'X Vel: %.2f' % vmx.getAHRS().GetVelocityX()
        cv.putText (img, xVelocityString, (30,110), cv.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv.LINE_AA)
        yDispString = 'Y Disp: %.2f' % vmx.getAHRS().GetDisplacementY()
        cv.putText (img, yDispString, (30,130), cv.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv.LINE_AA)
        xDispString = 'X Disp: %.2f' % vmx.getAHRS().GetDisplacementX()
        cv.putText (img, xDispString, (30,150), cv.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv.LINE_AA)

	#Update network tables
        ntNavxSensorTimestamp = vmx.getAHRS().GetLastSensorTimestamp()
        ntNavxAngle = vmx.getAHRS().GetAngle()
        ntNavxYVel = vmx.getAHRS().GetVelocityY()
        ntNavxXVel = vmx.getAHRS().GetVelocityX()
        ntNavxYDisp = vmx.getAHRS().GetDisplacementY()
        ntNavxXDisp = vmx.getAHRS().GetDisplacementX()
        ntCubeAngle = cubeAngle
        ntCudeDist = cubeDistance
        
        #Show image
        cv.imshow('My Webcam', img_contours)

        #Put frame in video stream
        cvsource.putFrame(img_contours)

        #Write image to file
        if writeVideo:
            imgout.write(img_contours)

        #check for ESC key press
        if cv.waitKey(1) == 27:
            break

    #Close all open windows
    cv.destroyAllWindows()

    #Close video file
    imgout.release()

    #Close the log file
    log_file.write('Run stopped on %s.' % datetime.datetime.now())
    

#define main function
def main():
    mainloop()

if __name__ == '__main__':
    main()

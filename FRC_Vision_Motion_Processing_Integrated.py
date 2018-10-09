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

mount_height = 16
fov_v = 42

#Set program flags
writeVideo = True

#Define contour detector function
def detect_contours(img_raw):

    #Set global variables
    global imgwidth
    
    #Set known values
    FOV_angle_in_degrees = 23.55
    width_of_target = 13
    height_of_object = 11
    
    #Set object test values
    minarea = 2000

    #Camera mounting values
    mount_angle = 20
    mount_height = 50
    
    #Blur image to remove noise
    blur = cv.GaussianBlur(img_raw.copy(),(5,5),0)
        
    #Convert from BGR to HSV colorspace
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

    #Define range of target color in HSV
    targetMin = (29, 32, 169)
    targetMax = (179, 255, 255)

    #Set pixels to white if in target HSV range, else set to black
    mask = cv.inRange(hsv, targetMin, targetMax)

    #Find contours in mask
    image, contours, hierarchy = cv.findContours(mask,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)

    #Find bounding rectangles of largest objects
    dist = -1
    angle = 0
    img_contours = cv.rectangle(img_raw,(0,0),(0,0),(0,0,255),2)

    if len(contours) > 0:
        mainContour = contours[0]

        #Get area of contour
        area = cv.contourArea(mainContour)

        #Check area before further processing
        if area > minarea:

            #Get bounding rectangle and draw on image
            x,y,w,h = cv.boundingRect(mainContour)
            img_contours = cv.rectangle(img_raw,(x,y),(x+w,y+h),(0,0,255),2)

            #Deal with integer problems
            w *= 1.0
            h *= 1.0

            #Based on the aspect ratio and comparison of the width and the height determine the height of the box, width, distance to it and angle of offset

            if h >= w:
                
                if (22.0 / (13 * math.sqrt(2)) - 0.005) < (h / w) and (h / w) < (22.0/13.0 + 0.005):
                    
                    #we have two stacked cubes
                    height_of_target = 22
                    width_of_target = (height_of_target / h) * w
                    
                elif (33.0/(13 * math.sqrt(2)) - 0.005) < (h / w) and ((h / w) < 33.0/13.0 + 0.005):
                    
                    #we have three stacked cubes
                    height_of_target = 33
                    width_of_target = (height_of_target / h) * w

                else:

                    #we have one cube on its 13x11 side
                    height_of_target = 13
                    width_of_target = (height_of_target / h) * w
                    
            elif w > h:
               
                if 11/(39 * math.sqrt(2)) < (h/w) and (h/w) < 11/39:

                    #we have three cubes side-by-side
                    height_of_target = 11
                    width_of_target = (height_of_target / h) * w

                elif 11/(26.0 * math.sqrt(2)) < (h/w) and (h/w) < 11/26:

                    #we have two cubes side-by-side
                    height_of_target = 11
                    width_of_target = (height_of_target / h) * w
                
                elif (11.0/(13.0 * math.sqrt(2))- 0.005) < (h/w) and (h/w) < (13.0/math.sqrt(290) + 0.006):

                    #we have one cube on the 13x13 side
                    height_of_target = 11
                    width_of_target = (height_of_target / h) * w

                elif (13.0/math.sqrt(290) - 0.006) < (h/w) and (h/w) < (11.0/13.0 + 0.005):

                    height_tester = 11

                    width_tester = (13.0/h) * w

                    area_tester = height_tester * width_tester

                    if 143.0 - 0.005 < area_tester and area_tester < 158.503 + 0.005:

                        height_of_target = 13
                        width_of_target = (height_of_target / h) * w

                    else:

                        height_of_target = 11
                        width_of_target = (height_of_target / h) * w
                    
                        
                elif (11.0/13.0 - 0.005) < (h/w) and (h/w) < 1.005:

                    #we have one cube on its 13x11 side
                    height_of_target = 13
                    width_of_target = (height_of_target / h) * w
                        
                else:
                    #there is no cube
                    height_of_target = 0
                    width_of_target = 0
            else:

                #there is no cube
                height_of_target = 0
                width_of_target = 0
                

            #calculate distance based off of height of cube (in inches)
            extraHeight = (height_of_target/h) * y
            distance = ((height_of_target + extraHeight) - mount_height)/math.tan(math.radians(fov_v/2.0)) * -1

            #calculate angle offset to center of cube (in degrees)
            angle = math.degrees(math.atan2((height_of_target/h) * (x + (w/2.0) - 160), distance))
            

    #return results
    return img_contours, distance, angle, height_of_target


#define main processing function
def mainloop():

    #Get currnt time as a string
    currentTime = time.localtime(time.time())
    timeString = str(currentTime.tm_year) + str(currentTime.tm_mon) + str(currentTime.tm_mday) + str(currentTime.tm_hour) + str(currentTime.tm_min)
    
    #Open a log file
    logFilename = '/data/Logs/Run_Log_' + timeString + '.txt'
    log_file = open(logFilename, 'w')
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
    cvsource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, imgwidth, imgheight, frames_per_sec)

    #Start processed Video Streaming Server
    cvMjpegServer = cs.MjpegServer("cvhttpserver", 8082)
    cvMjpegServer.setSource(cvsource)

    #Create blank image
    img = np.zeros(shape=(imgheight, imgwidth, 3), dtype=np.uint8)

    #Set video codec and create VideoWriter
    fourcc = cv.VideoWriter_fourcc(*'XVID')
    videoFilename = '/data/Match Videos/robotcam-' + timeString + '.avi'
    imgout = cv.VideoWriter(videoFilename,fourcc,20.0,(320,240))

    #Connect NetworkTables
    NetworkTables.initialize(server="10.41.21.2")
    navxTable = NetworkTables.getTable("navx")
    visionTable = NetworkTables.getTable("vision")
    smartDash = NetworkTables.getTable("SmartDashboard")

    
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
        img_contours, cubeDistance, cubeAngle, cubeHeight = detect_contours(img)

        #Put contour info on image
        if cubeDistance != -1:
            distanceString = 'Distance: %d' % cubeDistance
            cv.putText(img_contours, distanceString, (10,15), cv.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1,cv.LINE_AA)
            angleString = 'Offset angle: %d' % cubeAngle
            cv.putText(img_contours, angleString, (10,30), cv.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1,cv.LINE_AA)

        #Put IMU info on image
        angleString = 'Angle: %.2f' % vmx.getAHRS().GetAngle()
        cv.putText (img, angleString, (30,70), cv.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1,cv.LINE_AA)
        yVelocityString = 'Y Vel:  %.2f' % vmx.getAHRS().GetVelocityY()
        cv.putText (img, yVelocityString, (30,90), cv.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1,cv.LINE_AA)
        xVelocityString = 'X Vel: %.2f' % vmx.getAHRS().GetVelocityX()
        cv.putText (img, xVelocityString, (30,110), cv.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1,cv.LINE_AA)
        yDispString = 'Y Disp: %.2f' % vmx.getAHRS().GetDisplacementY()
        cv.putText (img, yDispString, (30,130), cv.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1,cv.LINE_AA)
        xDispString = 'X Disp: %.2f' % vmx.getAHRS().GetDisplacementX()
        cv.putText (img, xDispString, (30,150), cv.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1,cv.LINE_AA)

	#Update network tables
        navxTable.putNumber("SensorTimestamp", vmx.getAHRS().GetLastSensorTimestamp())
        navxTable.putNumber("DriveAngle", vmx.getAHRS().GetAngle())
        navxTable.putNumber("YVelocity", vmx.getAHRS().GetVelocityY())
        navxTable.putNumber("XVelocity", vmx.getAHRS().GetVelocityX())
        navxTable.putNumber("YDisplacement", vmx.getAHRS().GetDisplacementY())
        navxTable.putNumber("XDisplacement", vmx.getAHRS().GetDisplacementX())
        visionTable.putNumber("CubeAngle", cubeAngle)
        visionTable.putNumber("CubeDistance", cubeDistance)
        smartDash.putNumber("DriveAngle", vmx.getAHRS().GetAngle())
        smartDash.putNumber("CubeAngle", cubeAngle)
        smartDash.putNumber("CubeDistance", cubeDistance)
        
        #Show image
        cv.imshow('My Webcam', img_contours)

        #Put frame in video stream
        cvsource.putFrame(img_contours)

        #Write image to file
        if writeVideo:
            imgout.write(img_contours)

        #Check for stop code from robot
        if cv.waitKey(1) == 27:
            break
        #robotStop = visionTable.getNumber("RobotStop", 0)
        #if robotStop == 1:
        #    break

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

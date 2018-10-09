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
import logging

from cscore import CameraServer
from networktables import NetworkTables
from networktables.util import ntproperty
from time import sleep
from sys import exit

#Set up basic logging
logging.basicConfig(level=logging.DEBUG)

#Set image variables
imgwidth = 160
imgheight = 120
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
    minarea = 300

    #Camera mounting values
    mount_angle = 20
    mount_height = 50
    
    #Blur image to remove noise
    blur = cv.GaussianBlur(img_raw.copy(),(5,5),0)
        
    #Convert from BGR to HSV colorspace
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

    #Define range of Power Cube color in HSV
    targetMin = (25, 54, 161)
    targetMax = (38, 172, 254)

    #targetMin = (29, 32, 169)
    #targetMax = (179, 255, 255)

    #Define range of test object color in HSV
    #targetMin = (81, 14, 60)
    #targetMax = (117, 180, 213)

    #Set pixels to white if in target HSV range, else set to black
    mask = cv.inRange(hsv, targetMin, targetMax)

    #Find contours in mask
    image, contours, hierarchy = cv.findContours(mask,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)

    #Initialize some processing variables
    numTargets = 0
    distanceToCube = -1
    targetH = 0
    targetW = 0
    targetX = 0
    targetY = 0
    angleToCube = 0
    height_of_target = 0
    largestArea = 0   
    img_contours = cv.rectangle(img_raw,(0,0),(0,0),(0,0,255),2)
    largestContour = np.zeros((2,2))
    target_list = []
    loopCount = 0

    #Loop over all contours to find valid targets
    for testContour in contours:

        #Get area of contour
        area = cv.contourArea(testContour)

        #Check area before further processing
        if area > minarea:

            #Increment target count
            numTargets += 1

            #Check if this is the largest area
            if area > largestArea:
                
                if loopCount > 0:
                    target_list.append(largestContour)
                    
                largestContour = testContour
                largestArea = area
                
            else:

                if loopCount == 0:
                    target_list = [testContour]
                else:
                    target_list.append(testContour)

        loopCount += 1

                
    #If we found a contour, proceed with distance and angle calcs
    if largestArea > 0:
        
        #Get bounding rectangle and draw on image
        targetX,targetY,targetW,targetH = cv.boundingRect(largestContour)
        img_contours = cv.rectangle(img_raw,(targetX,targetY),(targetX+targetW,targetY+targetH),(0,0,255),2)

        #Deal with integer problems
        targetW *= 1.0
        targetH *= 1.0

        #Based on the aspect ratio and comparison of the width and the height determine the height of the box, width, distance to it and angle of offset
        if targetH >= targetW:
                
            if (22.0 / (13 * math.sqrt(2)) - 0.005) < (targetH / targetW) and (targetH / targetW) < (22.0/13.0 + 0.005):
                    
                #we have two stacked cubes
                height_of_target = 22
                width_of_target = (height_of_target / targetH) * targetW
                    
            elif (33.0/(13 * math.sqrt(2)) - 0.005) < (targetH / targetW) and ((targetH / targetW) < 33.0/13.0 + 0.005):
                    
                #we have three stacked cubes
                height_of_target = 33
                width_of_target = (height_of_target / targetH) * targetW

            else:

                #we have one cube on its 13x11 side
                height_of_target = 13
                width_of_target = (height_of_target / targetH) * targetW
                    
        elif targetW > targetH:
               
            if 11/(39 * math.sqrt(2)) < (targetH/targetW) and (targetH/targetW) < 11/39:

                #we have three cubes side-by-side
                height_of_target = 11
                width_of_target = (height_of_target / targetH) * targetW

            elif 11/(26.0 * math.sqrt(2)) < (targetH/targetW) and (targetH/targetW) < 11/26:

                #we have two cubes side-by-side
                height_of_target = 11
                width_of_target = (height_of_target / targetH) * targetW
                
            elif (11.0/(13.0 * math.sqrt(2))- 0.005) < (targetH/targetW) and (targetH/targetW) < (13.0/math.sqrt(290) + 0.005):

                #we have one cube on the 13x13 side
                height_of_target = 11
                width_of_target = (height_of_target / targetH) * targetW

            elif (13.0/math.sqrt(290) - 0.005) < (targetH/targetW) and (targetH/targetW) < (11.0/13.0 + 0.005):

                height_tester = 11

                width_tester = (13.0/targetH) * targetW

                area_tester = height_tester * width_tester

                if 143.0 - 0.005 < area_tester and area_tester < 158.503 + 0.005:

                    height_of_target = 13
                    width_of_target = (height_of_target / targetH) * targetW

                else:

                    height_of_target = 11
                    width_of_target = (height_of_target / targetH) * targetW
                    
                        
            elif (11.0/13.0 - 0.005) < (targetH/targetW) and (targetH/targetW) < 1.005:

                #we have one cube on its 13x11 side
                height_of_target = 13
                width_of_target = (height_of_target / targetH) * targetW
                        
            else:
                #there is no cube
                height_of_target = 0
                width_of_target = 0
        else:

            #there is no cube
            height_of_target = 0
            width_of_target = 0
                

        #calculate distance based off of height of cube (in inches)
        extraHeight = (height_of_target/targetH) * targetY
        distanceToCube = ((height_of_target + extraHeight) - mount_height)/math.tan(math.radians(fov_v/2.0)) * -1

        #calculate angle offset to center of cube (in degrees)
        angleToCube = math.degrees(math.atan2((height_of_target/targetH) * (targetX + (targetW/2.0) - 160), distanceToCube))


        #Loop over remaining contours and add bounding rectangle
        for target in target_list:

            if target.shape[0] > 10:

                #Get bounding rectangle and draw on image
                x,y,w,h = cv.boundingRect(target)
                img_contours = cv.rectangle(img_raw,(x,y),(x+w,y+h),(0,255,0),1)


    #return results
    return numTargets, img_contours, distanceToCube, angleToCube, height_of_target, targetH, targetW


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

    #Connect NetworkTables
    #NetworkTables.setIPAddress('10.41.21.2')
    #NetworkTables.setIPAddress('192.168.1.2')
    #NetworkTables.setClientMode()
    #NetworkTables.setTeam(4121)
    #NetworkTables.initialize()
    NetworkTables.initialize(server='10.41.21.2')
    navxTable = NetworkTables.getTable("navx")
    visionTable = NetworkTables.getTable("vision")
    smartDash = NetworkTables.getTable("SmartDashboard")

    #Initialize robot flags
    visionTable.putNumber("RobotStop", 0)
    navxTable.putNumber("ZeroGyro", 0)
        
    #Open connection to USB camera (video device 0)
    #camera = cs.UsbCamera("usbcam", 0)
    #camera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, imgwidth, imgheight, frames_per_sec)

    #Set up a camera server
    camserv = CameraServer.getInstance()
    camserv.enableLogging

    #Start capturing webcam video
    camera = camserv.startAutomaticCapture(dev=0, name="MainPICamera")
    camera.setResolution(imgwidth, imgheight)

    #Start raw Video Streaming Server
    #mjpegServer = cs.MjpegServer("httpserver", 8081)
    #mjpegServer.setSource(camera)

    #Define video sink
    #cvsink = cs.CvSink("cvsink")
    #cvsink.setSource(camera)
    cvsink = camserv.getVideo()

    #Define video source
    #cvsource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, imgwidth, imgheight, frames_per_sec)

    #Start processed Video Streaming Server
    #cvMjpegServer = cs.MjpegServer("cvhttpserver", 554)
    #cvMjpegServer.setSource(cvsource)

    #Create an output video stream
    outputStream = camserv.putVideo("ProcessCamera", imgwidth, imgheight)

    #Create blank image
    img = np.zeros(shape=(imgwidth, imgheight, 3), dtype=np.uint8)

    #Set video codec and create VideoWriter
    fourcc = cv.VideoWriter_fourcc(*'XVID')
    videoFilename = '/data/Match Videos/robotcam-' + timeString + '.avi'
    imgout = cv.VideoWriter(videoFilename,fourcc,20.0,(320,240))

    #Reset yaw gyro
    vmx.getAHRS().ZeroYaw()

    #Reset displacement
    vmx.getAHRS().ResetDisplacement()

    #Initialize vision variables
    targetHeight = 0
    targetWidth = 0

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
        numberOfTargets, img_contours, cubeDistance, cubeAngle, cubeHeight, targetHeight, targetWidth = detect_contours(img)
        
        if targetWidth > 0:
            aspectRatio = targetHeight/targetWidth
        else:
            aspectRatio = -9999

        #Put contour info on image
        #if cubeDistance != -1:
        #    distanceString = 'Target Distance: %d' % cubeDistance
        #    cv.putText(img_contours, distanceString, (10,15), cv.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1,cv.LINE_AA)
        #    angleString = 'Target Angle: %d' % cubeAngle
        #    cv.putText(img_contours, angleString, (10,30), cv.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1,cv.LINE_AA)

        #Put IMU info on image
        #angleString = 'Drive Angle: %.2f' % vmx.getAHRS().GetAngle()
        #cv.putText (img, angleString, (10,45), cv.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1,cv.LINE_AA)

	#Update network tables
        navxTable.putNumber("SensorTimestamp", vmx.getAHRS().GetLastSensorTimestamp())
        navxTable.putNumber("DriveAngle", round(vmx.getAHRS().GetAngle(), 2))
        navxTable.putNumber("YVelocity", round(vmx.getAHRS().GetVelocityY(), 4))
        navxTable.putNumber("XVelocity", round(vmx.getAHRS().GetVelocityX(), 4))
        navxTable.putNumber("YDisplacement", round(vmx.getAHRS().GetDisplacementY(), 2))
        navxTable.putNumber("XDisplacement", round(vmx.getAHRS().GetDisplacementX(), 2))
        visionTable.putNumber("NumberOfTargets", numberOfTargets)
        visionTable.putNumber("CubeAngle", round(cubeAngle, 2))
        visionTable.putNumber("CubeDistance", round(cubeDistance, 2))
        visionTable.putNumber("CubeHeight", round(cubeHeight, 2))
        smartDash.putNumber("DriveAngle", round(vmx.getAHRS().GetAngle(), 2))
        smartDash.putNumber("NumberofTargets", numberOfTargets)
        smartDash.putNumber("CubeAngle", round(cubeAngle, 2))
        smartDash.putNumber("CubeDistance", round(cubeDistance, 2))
        smartDash.putNumber("TargetHeight", round(targetHeight, 2))
        smartDash.putNumber("TargetWidth", round(targetWidth, 2))
        smartDash.putNumber("TargetAspectRatio", round(aspectRatio, 2))
        
        #Show image
        #cv.imshow('My Webcam', img_contours)

        #Put frame in video stream
        #cvsource.putFrame(img_contours)
        outputStream.putFrame(img_contours)

        #Write image to file
        if writeVideo:
            imgout.write(img_contours)

        #Check for gyro re-zero
        gyroInit = navxTable.getNumber("ZeroGyro", 0)
        if gyroInit == 1:
            vmx.getAHRS().ZeroYaw()
            navxTable.putNumber("ZeroGyro", 0)

        #Check for stop code from robot
        #if cv.waitKey(1) == 27:
        #    break
        robotStop = visionTable.getNumber("RobotStop", 0)
        if robotStop == 1:
            break

    #Close all open windows
    #cv.destroyAllWindows()

    #Close video file
    imgout.release()

    #Close the log file
    log_file.write('Run stopped on %s.' % datetime.datetime.now())
    log_file.close()
    

#define main function
def main():
    mainloop()

if __name__ == '__main__':
    main()

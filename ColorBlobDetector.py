#Imports
import cv2
import numpy as np


#Define blob detector function
def detect_blobs():
    cam = cv2.VideoCapture(0)
    
    #set video capture properties
    ret = cam.set(3,320)
    ret = cam.set(4,240)
    ret = cam.set(10,0.5)

    #start capturing and processing video frames
    while (True):

        #Grab a frame
        ret_val, img = cam.read()

        #Blur image to remove noise
        #Convert from BGR to HSV colorspace
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #Define range of target color in HSV
        targetMin = (21, 100, 110)
        targetMax = (143, 184, 240)

        #Set pixels to white if in target HSV range, else set to black
        mask = cv2.inRange(hsv, targetMin, targetMax)

        #Bitwise-AND of mask and target color - used for display
        res = cv2.bitwise_and(img, img, mask = mask)

        #Dilate makes the in range areas large
        #mask = cv2.dilate(mask, None, iterations=1)

        #Set up SimpleBlobDetector params
        params = cv2.SimpleBlobDetector_Params()

        #Set threshold
        params.minThreshold = 70
        params.maxThreshold = 100

        #Filter by area
        params.filterByArea = True
        params.minArea = 5000
        params.maxArea = 76800

        #Filter by circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1
        params.maxCircularity = .9

        #Filter by convexity
        params.filterByConvexity = False
        params.minConvexity = 0.75

        #Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.1

        #Create the detector
        detector = cv2.SimpleBlobDetector_create(params)

        #Detect the blobs
        reversemask = 255-mask
        keypoints = detector.detect(reversemask)

        if keypoints:
            print("Found blobs!")
        else:
            print("No blobs found")
            
        #Draw red circles
        im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        #Show blobs and mask
        cv2.imshow('My Webcam', im_with_keypoints)
        cv2.imshow('Mask', mask)
        cv2.imshow('Reverse Mask', reversemask)

        #check for ESC key press
        if cv2.waitKey(1) == 27:
            break

    #release webcam (stop video feed)
    cam.release()

    #close all open windows
    cv2.destroyAllWindows()



#define main function
def main():
    detect_blobs()

if __name__ == '__main__':
    main()

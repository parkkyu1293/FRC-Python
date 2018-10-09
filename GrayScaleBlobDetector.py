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

        #Convert from BGR to HSV colorspace
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 160
        params.maxThreshold = 240

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 50

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.5
        params.maxCircularity = 0.9

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.75
    
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.6
        params.maxInertiaRatio = 1.0

        #Create the detector
        detector = cv2.SimpleBlobDetector_create(params)

        #Detect the blobs
        keypoints = detector.detect(gray)

        #Draw red circles
        im_with_keypoints = cv2.drawKeypoints(gray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        #Show blobs
        cv2.imshow('My Webcam', im_with_keypoints)

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

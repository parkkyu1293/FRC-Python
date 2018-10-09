#!/usr/bin/env python3.5

#System imports
import sys
import os
import imp

#Setup paths
sys.path.append('/home/pi/.local/lib/python3.5/site-packages')
sys.path.append('/usr/local/lib/vmxpi/')

#Module imports
import numpy as np
import math
import datetime

from time import sleep
from sys import exit


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

    #Start main processing loop
    while (True):

        #Print gyro values
        yawString = 'Yaw:  %.2f' % vmx.getAHRS().GetYaw()
        print(yawString)
        pitchString = 'Pitch: %.2f' % vmx.getAHRS().GetPitch()
        print(pitchString)
        rollString = 'Roll:  %.2f' % vmx.getAHRS().GetRoll()
        print(rollString)
	
        #check for ESC key press
        if cv.waitKey(1) == 27:
            break

    #Close the log file
    log_file.write('Run stopped on %s.' % datetime.datetime.now())
    

#define main function
def main():
    mainloop()

if __name__ == '__main__':
    main()

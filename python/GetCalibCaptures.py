# -*- coding: utf-8 -*-
"""
Created on Tue Jul 17 13:35:19 2018

@author: Propietario
"""

import numpy as np
import cv2
import time

cap = cv2.VideoCapture(0)
cap.set(cv2.CV_CAP_PROP_FRAME_WIDTH,1920.0) #620 -- property 3
cap.set(cv2.CV_CAP_PROP_FRAME_HEIGHT,1080.0) #480 -- property 4
step=0

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',gray)
    time.sleep(1)
    write_name = 'calib_images/Camera/Calibracio20_07/Image'+str(step)+'.jpg'
#    write_name = 'calib_images/Laser/Calibracio20_07/Image'+str(step)+'.jpg'

    cv2.imwrite(write_name, frame)
    step=step+1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
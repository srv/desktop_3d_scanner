# -*- coding: utf-8 -*-
"""
Created on Mon Jun 18 16:04:54 2018

@author: Marta Pons Nieto
"""
import numpy as np
import cv2
import time

camera_matrix = np.load("cameraCalibMatrix.npy")
dist_coeffs = np.load("cameraCalibDistCoeff.npy")
cap = cv2.VideoCapture(0)

while cap.isOpened()==False:
    cap.open()
    
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
marker_length=0.03 #in meters 

while(True):
    while cap.isOpened()==False:
        cap.open()
#        print("IS CLOSED")
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if ret==True:
        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray,dictionary)

        if len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(gray,corners,ids)

                i=0
                while(i<len(ids)):
                    rvecs,tvecs,_objPoints  = cv2.aruco.estimatePoseSingleMarkers(corners[i],marker_length,camera_matrix,dist_coeffs)
                    cv2.aruco.drawAxis(gray,camera_matrix,dist_coeffs,rvecs,tvecs,0.05)
                    i = i+1
#                    planeTable = rvecs 
#                    planeTable_D = np.sum(rvecs*tvecs)
#                    print(planeTable)
#                    print(planeTable_D)
        # Display the resulting frame
        cv2.imshow('frame',gray)
        
        if cv2.waitKey(10):
            if 0xFF == ord('q'):
                break
    else:
        print("Ret")
        print(ret)
        print("Cam opened?")
        print(cap.isOpened())
        cap.release()
        cv2.destroyAllWindows()
        cap = cv2.VideoCapture(0)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

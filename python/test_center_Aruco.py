# -*- coding: utf-8 -*-
"""
Created on Mon Jun 18 16:04:54 2018

@author: Marta Pons Nieto
"""
import numpy as np
import cv2
import glob

def fitPlane(norm,point):
    A=norm[0]
    B=norm[1]
    C=norm[2]
    D=-np.sum(norm*point)
    return(A,B,C,D)

camera_matrix = np.load("cameraCalibMatrix.npy")
dist_coeffs = np.load("cameraCalibDistCoeff.npy")
cap = cv2.VideoCapture(0)

while cap.isOpened()==False:
    cap.open()
    
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
marker_length=0.03 #in meters 
#images = glob.glob('C:\Users\Propietario\Documents\GitHub\desktop_3d_scanner\python\DataOffline\*.png')

while(True):
    while cap.isOpened()==False:
        cap.open()
#        print("IS CLOSED")
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if ret==True:
        frame = cv2.imread('C:\Users\Propietario\Documents\GitHub\desktop_3d_scanner\python\DataOffline\TableCalib.jpg')

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray,dictionary)
        
        tvecs5=np.array([[[0,0,0]]])
        tvecs42=np.array([[[0,0,0]]])
        tvecs27=np.array([[[0,0,0]]])
        tvecs18=np.array([[[0,0,0]]])
        
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(gray,corners,ids)
            i=0
            while(i<len(ids)):
                rvecs,tvecs,_objPoints  = cv2.aruco.estimatePoseSingleMarkers(corners[i],marker_length,camera_matrix,dist_coeffs)
                cv2.aruco.drawAxis(gray,camera_matrix,dist_coeffs,rvecs,tvecs,0.05)
                if (ids[i]==5):
                    R,_=cv2.Rodrigues(rvecs)
                    t = np.array([-0.09,0,0])
                    tvecs5=np.matmul(R,t.T)+tvecs
                if (ids[i]==42):
                    R,_=cv2.Rodrigues(rvecs)
                    t = np.array([+0.09,0,0])
                    tvecs42=np.matmul(R,t.T)+tvecs
                if (ids[i]==27):
                    R,_=cv2.Rodrigues(rvecs)
                    t = np.array([0,-0.09,0])
                    tvecs27=np.matmul(R,t.T)+tvecs
                if (ids[i]==18):
                    R,_=cv2.Rodrigues(rvecs)
                    t = np.array([0,+0.09,0])
                    tvecs18=np.matmul(R,t.T)+tvecs

                i=i+1
                  
            tvecs = (tvecs5+tvecs42+tvecs27+tvecs18)/len(ids) #si té tvecs més petit té més prioritat
            norm = R[0:3,2]
            planeTable=fitPlane(norm,tvecs[0,0,0])
            cv2.aruco.drawAxis(gray,camera_matrix,dist_coeffs,rvecs,tvecs,0.05)
            
        # Display the resulting frame
        cv2.imshow('frame',gray)
        write_name = 'DataOffline/CalibracioAruco.jpg'
        cv2.imwrite(write_name, frame)
        
        
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

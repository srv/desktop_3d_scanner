# -*- coding: utf-8 -*-
"""
Created on Thu Jul 12 11:13:44 2018

@author: Propietario
"""
import cv2
import numpy as np

def intersection(plane,point,mtx):
    fx = mtx[0,0]
    fy = mtx[1,1]
    cx = mtx[0,2]
    cy = mtx[1,2]
    rt = [(point[0]-cx)/fx,(point[1]-cy)/fy,1]
    t = - plane[3]/((rt[0]*plane[0])+(rt[1]*plane[1])+(plane[2])) 
    point3D = np.array([[rt[0]*t, rt[1]*t,rt[2]*t]])
    return(point3D)

def calculate_rotation(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray,dictionary)
    marker_length=0.03 #in meters
        
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
        
#        norm = R[0:3,2]
#        planeTable=fitPlane(norm,tvecs[0,0,0])
        cv2.aruco.drawAxis(gray,camera_matrix,dist_coeffs,rvecs,tvecs,0.05)
        
    return (R,tvecs)
        
    
def rotate_points(frame,points):
    calculate_rotation(frame)
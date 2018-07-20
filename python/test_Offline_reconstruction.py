# -*- coding: utf-8 -*-
"""
Created on Tue Jul 17 19:40:21 2018

@author: Propietario
"""

from laser_detector import detect_laser_subpixel
import cv2
import numpy as np
from math import isnan
import glob
import serial
import time
from matplotlib import pylab
from mpl_toolkits import mplot3d
from pointcloud_viewer import ReconstructionViewer


def outOfBounds(point,tvec):
    #Si la x esta fuera de los limites de la mesa
    if point[0]>tvec[0]+0.15:
        return True
    elif point[0]<tvec[0]-0.15:
        return True
    #O la coordenada y esta fuera de los limites
    elif point[1]>tvec[1]+0.15:
        return True
    elif point[1]<tvec[1]+0.15:
        return True
    else:
        return False
    #Sino se considerara como un punto acceptable


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
        cv2.aruco.drawAxis(gray,camera_matrix,dist_coeffs,rvecs,tvecs,0.05)

        # Display the resulting frame
        cv2.imshow('Tabe Calib',gray)
#        norm = R[0:3,2]
#        planeTable=fitPlane(norm,tvecs[0,0,0])
        cv2.aruco.drawAxis(gray,camera_matrix,dist_coeffs,rvecs,tvecs,0.05)

    return (R,tvecs)


#def rotate_points(frame,points):
#    calculate_rotation(frame)

#------------------------------MAIN--------------------------------------------
camera_matrix = np.load("cameraCalibMatrix3.npy")
dist_coeffs = np.load("cameraCalibDistCoeff3.npy")
laser_plane = np.load("LaserPlane3.npy")

ser = serial.Serial()
ser.port = "COM12"
ser.baudrate = 9600
ser.timeout = 1


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Obimages = glob.glob('~/git/3d_rotating_table_laser/python/DataOffline\*.jpg')
# Tableimages = glob.glob('~/git/3d_rotating_table_laser/python/DataOffline/Table\*.jpg')
Obimages = glob.glob('~/git/3d_rotating_table_laser/python/DataOffline\*.jpg')
Tableimages = glob.glob('~/git/3d_rotating_table_laser/python/DataOffline/Table\*.jpg')
#Obimages = glob.glob('~/git/3d_rotating_table_laser/laser_calibration\*.jpg')
kernel = np.array([[0.000003, 0.000229, 0.005977, 0.060598, 0.24173, 0.382925, 0.24173, 0.060598, 0.005977, 0.000229, 0.000003]], np.float32)
threshold = 0.1
window_size = 7

detected_points = np.array([[],[],[]]).T
# Inicialment la taula té rotació zero i translació zero
table_1to2 = np.identity(4)
table_2to1 = np.identity(4)

viewer  = ReconstructionViewer()


# La matriu de rotació d'un pas de motor (4x4) angle=1.125 Rot z
table_increment = np.array([[0.999807,-0.01963,0,0],
                       [0.01963,0.999807,0,0],
                       [0,0,1,0],
                       [0,0,0,1]])

# La transformació del sistema de coordenades càmera a la taula (calibració, 4x4)
camera_to_table_1 = np.identity(4)
table_1_to_camera = np.identity(4)

camera_to_table_1[0:3,0:3]=np.array([[-0.20223907, -0.91075492,  0.36003449],
       [-0.63157544, -0.15967696, -0.75869344],
       [ 0.748473  , -0.3808264 , -0.54291752]])
camera_to_table_1[0:3,3]= np.array([[-0.38777067, -0.13432474,  0.92294953]])

table2camera = np.array([[-0.2022,   -0.6316,    0.7485, -0.8541 ],
                        [-0.9108, -0.1597, -0.3808, -0.0231],
                        [ 0.3600, -0.7587, -0.5429,  0.5388],
                        [ 0, 0, 0, 1]])
tvec = np.array([-0.38777067, -0.13432474,  0.92294953])


step=1
while step<100:

    fname = '/home/miquel/git/3d_rotating_table_laser/python/DataOffline/step_num_table'+str(step)+'.jpg'
    print fname
    frameTable = cv2.imread(fname)

    if step ==1:
        R,tvec = calculate_rotation(frameTable)
        camera_to_table_1[0:3,0:3]=R
        camera_to_table_1[0:3,3]=tvec

    fname = '/home/miquel/git/3d_rotating_table_laser/python/DataOffline/step_num'+str(step)+'.jpg'
    frame = cv2.imread(fname)
    h, w = frame.shape[:2]

    # undistort
    newcamera, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w,h), 0)
    frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None, newcamera)

    ret=True


    #            #Perform a step
    #            ser.write("1")
    #            time.sleep(1)

    # Capture frame-by-frame
    #            ret, frame = cap.read()

    if ret==True:

        subpixel_peaks = detect_laser_subpixel(frame, kernel, threshold, window_size)
        subpixel_peaks= np.expand_dims(subpixel_peaks, 1)
        rectified_points = subpixel_peaks
#        rectified_points = cv2.undistortPoints(subpixel_peaks, camera_matrix, dist_coeffs)

        laser_points = np.array([[],[],[]]).T
        for p in rectified_points:
            x = p[0,0]
            y = p[0,1]

            table_1_to_camera = np.linalg.inv(camera_to_table_1)
            table_2_to_camera = np.matmul(table_2to1, table_1_to_camera)

            if not isnan(x) and not isnan(y):
                frame = cv2.circle(frame,(int(y),int(x)),15,(0,0,255),1)
                # cv2.imshow('Laser Image', frame)
                #Interseccion between planeLaser and point (u,v)
                new_point = intersection(laser_plane,(x,y),camera_matrix)
                new_point = table_2_to_camera[:3,:3] * new_point + table_2_to_camera[:3,3:]
                laser_points=np.concatenate((laser_points,new_point))

        viewer.append(laser_points, np.identity(4))
        viewer.drawnow()

        table_1to2 = np.matmul(table_1to2, table_increment)
        table_2to1 = np.linalg.inv(table_1to2)

        step=step+1

print "Proceso acabado"
viewer.run()
#else:
#    cv2.destroyAllWindows()






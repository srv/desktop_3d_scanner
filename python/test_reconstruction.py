# -*- coding: utf-8 -*-
"""
Created on Thu Jun 21 19:17:21 2018

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

images = glob.glob('C:\Users\Propietario\Documents\GitHub\desktop_3d_scanner\laser_calibration\*.jpg')
kernel = np.array([[0.000003, 0.000229, 0.005977, 0.060598, 0.24173, 0.382925, 0.24173, 0.060598, 0.005977, 0.000229, 0.000003]], np.float32)
threshold = 0.1
window_size = 7

detected_points = np.array([[],[],[]]).T
# Inicialment la taula té rotació zero i translació zero
table_postition = np.identity(4)

viewer  = ReconstructionViewer()


# La matriu de rotació d'un pas de motor (4x4)
table_increment = np.array([[0.999807,-0.01963,0,0],
                       [0.01963,0.999807,0,0],
                       [0,0,1,0],
                       [0,0,0,1]])

# La transformació del sistema de coordenades càmera a la taula (calibració, 4x4)
camera2table = np.identity(4)

#camera2table[0:3,0:3]=np.array([[ 0.37572923,  0.89664238,  0.2342221 ],
#       [ 0.62754291, -0.06019223, -0.77625176],
#       [-0.68192187,  0.4386449 , -0.58529754]])
camera2table[0:3,0:3]=np.array([[-0.20223907, -0.91075492,  0.36003449],
       [-0.63157544, -0.15967696, -0.75869344],
       [ 0.748473  , -0.3808264 , -0.54291752]])
#camera2table[0:3,3]=np.array([[[-0.3379647 , -0.12153296,  0.87844024]]])
camera2table[0:3,3]=np.array([[[-0.38777067, -0.13432474,  0.92294953]]])
  

cap = cv2.VideoCapture(0)
ret, frame = cap.read()
            
if ret==True:
    R,tvec = calculate_rotation(frame)

try: 
    ser.open()
    print ser.portstr       # check which port was really used
except Exception, e:
    print "error open serial port: " + str(e)
    exit()

if ser.isOpen():

    try:
        ser.flushInput() #flush input buffer, discarding all its contents
        ser.flushOutput()#flush output buffer, aborting current output 
                             #and discard all that is in buffer
        response = ser.readline()
        print("read data: " + response) #Check information from Arduino
        time.sleep(0.5)
        response=1
        
        step=0
        while step<1:
#        for fname in images:
#            frame = cv2.imread(fname)
#            ret=True
            
            #Perform a step    
            ser.write("1") 
            time.sleep(1)
            
            
#            while (response is not 1):
#                response = ser.readline()
#                print("read data: " + response) #Check information from Arduino
#                time.sleep(0.5)
#            response = 0    
            
            # Capture frame-by-frame
            ret, frame = cap.read()
            
            if ret==True:
                cv2.imshow('Laser Image', frame)
                subpixel_peaks = detect_laser_subpixel(frame, kernel, threshold, window_size)
#                rectified_points = subpixel_peaks
                subpixel_peaks= np.expand_dims(subpixel_peaks, 1)
                rectified_points = subpixel_peaks
                rectified_points = cv2.undistortPoints(subpixel_peaks, camera_matrix, dist_coeffs)
                
                laser_points = np.array([[],[],[]]).T
                for p in rectified_points:
                    x = p[0,0]
                    y = p[0,1]
                    if not isnan(x) and not isnan(y):
                        frame = cv2.circle(frame,(int(y),int(x)),15,(0,0,255),1)
                        cv2.imshow('Laser Image', frame)
                        #Interseccion between planeLaser and point (u,v) 
                        new_point = intersection(laser_plane,(x,y),camera_matrix)
                        laser_points=np.concatenate((laser_points,new_point))
                        
                  # Per transformar els punts respecte a la càmara a punts
                  # respecte la taula, cal aplicar la transformació
#                table2points = -camera2table*table_postition
                table2points = np.matmul(-camera2table,table_postition)

                viewer.append(laser_points, table2points)
                viewer.drawnow()
                
                
                detected_points= np.concatenate((detected_points,laser_points))
                
                
                table_postition = np.matmul(table_postition, table_increment)
                
                print step
                
                step=step+1
            else:
                cap.release()
                cv2.destroyAllWindows()
                cap = cv2.VideoCapture(0)
                _,frame = cap.read()
                
                  
        ser.close()
        print "Bucle acabado"
        viewer.run()



    except Exception, e1:
        print "error communicating...: " + str(e1)

else:
    print "cannot open serial port "


    
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 18 13:12:03 2018

@author: Propietario
"""

import numpy as np
 
# Read from file
ra = np.load("cameraCalib.npy")
dist_coeff = [[0.12384288610115032, -0.2424483657117197, -0.002006010882372743, -0.00042451446778679077, 0.07986794336491727]] 
camera_matrix = [[1395.6279212009845, 0.0, 970.8046805524448], [0.0, 1393.6892151878556, 532.5444388658885], [0.0, 0.0, 1.0]]
print(dist_coeff)
print(camera_matrix)
rb = np.load("cameraCalibMatrix.npy")
rc = np.load("cameraCalibDistCoeff.npy")
print(rb)
print(rc)

def saveCoefficients(mtx, dist):
    cv_file = cv2.FileStorage("calib_images/calibrationCoefficients.yaml", cv2.FILE_STORAGE_WRITE)
    cv_file.write("camera_matrix", mtx)
    cv_file.write("dist_coeff", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()
    
def loadCoefficients():
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage("calib_images/calibrationCoefficients.yaml", cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("camera_matrix").mat()
    dist_matrix = cv_file.getNode("dist_coeff").mat()

    # Debug: print the values
    # print("camera_matrix : ", camera_matrix.tolist())
    # print("dist_matrix : ", dist_matrix.tolist())

    cv_file.release()
    return [camera_matrix, dist_matrix]
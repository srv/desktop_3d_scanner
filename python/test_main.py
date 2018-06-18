# -*- coding: utf-8 -*-
"""
Created on Fri Jun 15 11:51:00 2018

@author: Marta Pons
@brief: Laser calibration: obtain the Laser plane 
"""
from laser_detector import detect_laser_subpixel
import cv2

images = glob.glob('C:\Users\Propietario\Documents\GitHub\desktop_3d_scanner\laser_calibration\*.jpg')
kernel = np.array([[0.000003, 0.000229, 0.005977, 0.060598, 0.24173, 0.382925, 0.24173, 0.060598, 0.005977, 0.000229, 0.000003]], np.float32)
threshold = 0.1
window_size = 7

for fname in images:
    img = cv2.imread(fname)
    "img = undistortImage(imOrig, cameraParams);
    subpixel_peaks = detect_laser_subpixel(img, kernel, threshold, window_size)
    "solvePNP
    "planeCamera
    "Intersect between 2D and 3D
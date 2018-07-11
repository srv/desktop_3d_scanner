#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jun  7 11:31:52 2018

@author: miquel
"""

from laser_detector import detect_laser_subpixel
import cv2
import numpy as np
import glob
from math import isnan
    
images = glob.glob('C:\Users\Propietario\Documents\GitHub\desktop_3d_scanner\laser_calibration\bcap1.jpg')
img = cv2.imread(images)
kernel = np.array([[0.000003, 0.000229, 0.005977, 0.060598, 0.24173, 0.382925, 0.24173, 0.060598, 0.005977, 0.000229, 0.000003]], np.float32);
threshold = 0.1;
window_size = 7;
subpixel_peaks = detect_laser_subpixel(img, kernel, threshold, window_size);

cv2.namedWindow('Laser Image', cv2.WINDOW_KEEPRATIO)
for p in subpixel_peaks:
    x, y = p[0]
    if not isnan(x) and not isnan(y):
        img = cv2.circle(img,(int(y),int(x)),15,(0,0,255),1)
cv2.imshow('Laser Image', img)
cv2.waitKey(10000)
cv2.destroyAllWindows()
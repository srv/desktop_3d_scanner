#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jun  7 16:55:01 2018

@author: miquel
"""

import cv2
import numpy as np

EPS = 1e-7

def detect_laser_subpixel(img, kernel, threshold, window_size):
    #%%
    b,g,r = cv2.split(img)
    laser_ch = g.astype(float)
    laser_ch = laser_ch/laser_ch.max()
    c = cv2.filter2D(laser_ch, -1, kernel)
    m = c.argmax(1)
    peaks = []
    for row in range(img.shape[0]):
        col = m[row] - (kernel.size-1)/2
        value = laser_ch[row, col]
        if value > threshold:
            p = np.array([[row, col]])
            peaks.append(p)
    #%%
    # Get subpixel accuracy: Compute the Gaussian approximation or weight mass
    # centre of the surrounding pixels for each detected peak:
    subpixel_peaks = np.array([[],[]]).T
    for point in peaks:
        row, col = point[0]
        value = np.log(g[row, col])
        value_prev = np.log(g[row,col-1])
        value_next = np.log(g[row, col+1])
        offset = 0
        if value_prev+value_next-2*value > EPS or  value_prev+value_next-2*value < -EPS:
            offset = 1/2*((value_prev-value_next)/(value_prev+value_next-2*value))
        if abs(offset) > 1:
            mx = 0.0
            m = 0.0
            window = np.arange(-(window_size-1)/2, (window_size+1)/2, 1)
            for k in window:
                if col+k > laser_ch.shape[1]:
                    continue
                if col+k < 0:
                    continue
                value_k = float(laser_ch[row, col+k])
                mx = mx + value_k*float(k)
                m = m + value_k
            offset = mx / m
        peak = np.array([[float(row), float(col) + offset]], np.float32)
        subpixel_peaks=np.concatenate((subpixel_peaks,peak))
    #%%
    return subpixel_peaks
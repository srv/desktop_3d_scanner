#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jun  7 19:48:33 2018

@author: miquel
"""

import sys
from PyQt4 import QtGui, QtCore
import cv2

class QtCapture(QtGui.QWidget):
    def __init__(self, *args):
        super(QtGui.QWidget, self).__init__()

        self.fps = 24
        self.cap = cv2.VideoCapture(*args)
        self.isCapturing = False
        self.ith_frame = 1
        
        self.video_frame = QtGui.QLabel()
        lay = QtGui.QVBoxLayout()
        lay.setMargin(0)
        lay.addWidget(self.video_frame)
        self.setLayout(lay)

    def setFPS(self, fps):
        self.fps = fps

    def nextFrameSlot(self):
        ret, frame = self.cap.read()

        # Save images if isCapturing
        if self.isCapturing:
            cv2.imwrite('img_%05d.jpg'%self.ith_frame, frame)
            self.ith_frame += 1

        # My webcam yields frames in BGR format
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = QtGui.QImage(frame, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(img)
        self.video_frame.setPixmap(pix)

    def start(self):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.nextFrameSlot)
        self.timer.start(1000./self.fps)

    def stop(self):
        self.timer.stop()

    def capture(self):
        if not self.isCapturing:
            self.isCapturing = True
        else:
            self.isCapturing = False

    def deleteLater(self):
        self.cap.release()
        super(QtGui.QWidget, self).deleteLater()


class ControlWindow(QtGui.QWidget):
    def __init__(self):
        QtGui.QWidget.__init__(self)
        self.capture = None

        self.start_button = QtGui.QPushButton('Start')
        self.start_button.clicked.connect(self.startCapture)
        self.quit_button = QtGui.QPushButton('End')
        self.quit_button.clicked.connect(self.endCapture)
        self.end_button = QtGui.QPushButton('Stop')
        self.capture_button = QtGui.QPushButton('Capture')
        self.capture_button.clicked.connect(self.saveCapture)
        self.status_label = QtGui.QLabel('Status')
        self.status_label.setText("Status: Init")
        
        self.capture = QtCapture(0)
        self.capture.setFPS(1)
        #self.capture.setParent(self)
        self.capture.setWindowFlags(QtCore.Qt.Tool)
        self.end_button.clicked.connect(self.capture.stop)
        self.capture.start()
        self.capture.show()
        
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)

        hbox = QtGui.QHBoxLayout(self)
        vbox = QtGui.QVBoxLayout(self)
        vbox.addWidget(self.start_button)
        vbox.addWidget(self.end_button)
        vbox.addWidget(self.quit_button)
        vbox.addWidget(self.capture_button)
        vbox.addWidget(self.status_label)
        vbox.addSpacerItem(spacerItem)
        hbox.addWidget(self.capture)
        hbox.addLayout(vbox)

        self.setLayout(hbox)
        self.setWindowTitle('Control Panel')
        self.setGeometry(100,100,200,200)
        self.show()

    def startCapture(self):
        if not self.capture:
            self.capture = QtCapture(0)
            self.end_button.clicked.connect(self.capture.stop)
            self.capture.setFPS(1)
            self.capture.setParent(self)
            self.capture.setWindowFlags(QtCore.Qt.Tool)
        self.capture.start()
        self.capture.show()

    def endCapture(self):
        self.capture.deleteLater()
        self.capture = None
        sys.exit()

    def saveCapture(self):
        if self.capture:
            self.capture.capture()

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = ControlWindow()
    sys.exit(app.exec_())
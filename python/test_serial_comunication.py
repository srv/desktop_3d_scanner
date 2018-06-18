# -*- coding: utf-8 -*-
"""
Created on Fri Jun 15 15:35:06 2018

@author: Propietario
"""

#!/usr/bin/python

import serial
import time
import cv2
#initialization and open the port

#possible timeout values:
#    1. None: wait forever, block call
#    2. 0: non-blocking mode, return immediately
#    3. x, x is bigger than 0, float allowed, timeout block call

ser = serial.Serial()
ser.port = "COM12"
ser.baudrate = 9600
#ser.bytesize = serial.EIGHTBITS #number of bits per bytes
#ser.parity = serial.PARITY_NONE #set parity check: no parity
#ser.stopbits = serial.STOPBITS_ONE #number of stop bits
##ser.timeout = None          #block read
ser.timeout = 1              #non-block read
##ser.timeout = 2              #timeout block read
#ser.xonxoff = False     #disable software flow control
#ser.rtscts = False     #disable hardware (RTS/CTS) flow control
#ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
#ser.writeTimeout = 2     #timeout for write

cap = cv2.VideoCapture(0)
 
#ser = serial.Serial(port, baud, timeout=1)

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
        
        
        i=0
        while i<10:
                
            ser.write("0")
            time.sleep(1)
            
            #Wait untill STEP IS DONE
#            response = ser.readline()
#            print("read data: " + response) #Check information from Arduino
#            time.sleep(0.5)
            
            # Capture frame-by-frame
            ret, frame = cap.read()
            
            i=i+1
            print(i)
        
        ser.close()
    except Exception, e1:
        print "error communicating...: " + str(e1)

else:
    print "cannot open serial port "
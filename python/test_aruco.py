import numpy as np
import cv2
import time

cap = cv2.VideoCapture(0)
while cap.isOpened()==False:
    cap.open()
    
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
marker_length=0.3 

while(True):
    while cap.isOpened()==False:
        cap.open()
#        print("IS CLOSED")
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if ret==True:
        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        res = cv2.aruco.detectMarkers(gray,dictionary)
        print(res[0])

        if len(res[0]) > 0:
#            print(res[0])
            cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])
#            rvecs,tvecs = cv2.aruco.estimatePoseSingleMarkers(res[0],marker_length,cameraMatrix,distCoeffs)
            
        # Display the resulting frame
        cv2.imshow('frame',gray)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("Ret")
        print(ret)
        print("Cam opened?")
        print(cap.isOpened()) 

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

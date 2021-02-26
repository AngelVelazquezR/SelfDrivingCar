#!/usr/bin/env python3

import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt

cap = cv2.VideoCapture(1)
cont=0

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    copia = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    #ret, corners = cv2.findChessboardCorners(gray, (9,6), None)
    ret, corners = cv2.findCirclesGrid(gray,(11,4),
                                             flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
                                             blobDetector=cv2.SimpleBlobDetector_create(cv2.SimpleBlobDetector_Params()))
    print(corners)

    # If found, add object points, image points
    if ret == True:
        # Draw and display the corners
        cv2.drawChessboardCorners(frame, (9,6), corners, ret)
    

    # Display the resulting frame
    cv2.imshow('frame',frame )

    k = cv2.waitKey(33)
    if k == 27:
        break
    if k == 86 or k == 85 or k == 81 or k == 83:
        cv2.imwrite( "../figs/images/calibracion_"+str(cont)+".png", cv2.cvtColor(copia, cv2.COLOR_RGB2BGR))
        print("Guardado ",cont)
        cont=cont+1


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


    

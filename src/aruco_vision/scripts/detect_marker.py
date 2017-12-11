#!/usr/bin/env python

import numpy as np
import cv2
import cv2.aruco as aruco

calib = np.load('calib2.npz')
mtx, dist = calib['cm'], calib['dc']

print('mtx: {}, dist: {}'.format(mtx, dist))

markerLength = 39 # millimetres

cap = cv2.VideoCapture(2)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    print(frame.shape)
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters =  aruco.DetectorParameters_create()

    #print(parameters)

    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''
        #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(corners)

    # It's working.
    # my problem was that the cellphone put black all around it. The alrogithm
    # depends very much upon finding rectangular black blobs

    gray = aruco.drawDetectedMarkers(gray, corners, ids, (0,255,0))

    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, dist)
    if rvecs is not None:
        print('{} found'.format(rvecs.size))
        gray = aruco.drawAxis(gray, mtx, dist, rvecs[0], tvecs[0], 100)

    #print(rejectedImgPoints)
    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

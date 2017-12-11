#!/usr/bin/env python

import cv2
import numpy as np

calib = np.load('calibs/calib.npz')
calibStereo = np.load('calibs/calib-stereo.npz')

mtx1,dist1 = calib['mtx1'],calib['dist1']
mtx2,dist2 = calib['mtx2'],calib['dist2']

R1,P1 = calibStereo['R1'],calibStereo['P1']
R2,P2 = calibStereo['R2'],calibStereo['P2']

capL = cv2.VideoCapture(0)
capR = cv2.VideoCapture(2)

_,img = capL.read()
height,width,_ = img.shape

map1x, map1y = cv2.initUndistortRectifyMap(mtx1, dist1, R1, P1, (width, height), cv2.CV_32FC1)
map2x, map2y = cv2.initUndistortRectifyMap(mtx2, dist2, R2, P2, (width, height), cv2.CV_32FC1)

while True:
    _,frameL = capL.read()
    _,frameR = capR.read()

    vis = np.concatenate((frameL, frameR), axis=1) # combine horiz
    vis = cv2.resize(vis, (960,270))
    cv2.imshow('frame', vis)

    imgL = cv2.remap(frameL, map1x, map1y, cv2.INTER_LINEAR)
    imgR = cv2.remap(frameR, map2x, map2y, cv2.INTER_LINEAR)

    vis = np.concatenate((imgL, imgR), axis=1) # combine horiz
    vis = cv2.resize(vis, (960,270))
    cv2.imshow('frame2', vis)


    if cv2.waitKey(1) == ord('q'):
        break

capL.release()
capR.release()
cv2.destroyAllWindows()

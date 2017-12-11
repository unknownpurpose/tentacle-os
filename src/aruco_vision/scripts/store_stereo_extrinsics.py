#!/usr/bin/env python

import cv2
import numpy as np
import glob
import pdb

from helpers import getCornersFromImage

calib = np.load('calibs/calib.npz')

mtx1,dist1 = calib['mtx1'],calib['dist1']
mtx2,dist2 = calib['mtx2'],calib['dist2']

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(5,7,.025,.0125,dictionary)
boardIds,boardCorners = getCornersFromImage(board.draw((5*100,7*100)), board, dictionary)
boardCorners = np.hstack((boardCorners, np.zeros((len(boardCorners),1))))

imgPaths1 = glob.glob('images/cap0-*.png')
imgPaths2 = glob.glob('images/cap2-*.png')

allCorners1 = []
allCorners2 = []
allObjectCorners = []

for i in range(len(imgPaths1)):
    img1 = cv2.imread(imgPaths1[i])
    img2 = cv2.imread(imgPaths2[i])
    ids1,corners1 = getCornersFromImage(img1, board, dictionary)
    ids2,corners2 = getCornersFromImage(img2, board, dictionary)

    foundIds = np.intersect1d(ids1, ids2)

    fc1 = corners1[np.in1d(ids1,foundIds)]
    fc2 = corners2[np.in1d(ids2,foundIds)]
    refc = boardCorners[np.in1d(boardIds,foundIds)]

    allCorners1.append(fc1.astype('float32'))
    allCorners2.append(fc2.astype('float32'))
    allObjectCorners.append(refc.astype('float32'))

# get image size from a single image
img = cv2.imread('images/cap0-0.png')
height,width,_ = img.shape

_,_,_,_,_,R,T,_,_ = cv2.stereoCalibrate(allObjectCorners, allCorners1, allCorners2, mtx1, dist1, mtx2, dist2, (height,width), flags=cv2.CALIB_FIX_INTRINSIC)
R1,R2,P1,P2,_,_,_ = cv2.stereoRectify(mtx1, dist1, mtx2, dist2, (height, width), R, T)

np.savez('calibs/calib-stereo.npz',R=R,T=T,R1=R1,R2=R2,P1=P1,P2=P2)

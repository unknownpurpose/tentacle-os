#!/usr/bin/env python

import cv2
import numpy as np
import glob
import pdb

calib = np.load('calib.npz')

mtx1,dist1 = calib['mtx1'],calib['dist1']
mtx2,dist2 = calib['mtx2'],calib['dist2']

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(5,7,.025,.0125,dictionary)
numCorners = len(board.chessboardCorners)

def getCornersFromImageWithPath(imgPath, board, dictionary):
    img = cv2.imread(imgPath)
    mCorners, mIds, _ = cv2.aruco.detectMarkers(img, dictionary)
    _, boardCorners, _ = cv2.aruco.interpolateCornersCharuco(mCorners, mIds, img, board)
    boardCorners = np.reshape(boardCorners, (len(boardCorners), 2)) # from (N, 1, 2) to (N, 2)
    return boardCorners

allCorners1 = []
allCorners2 = []
imgPaths1 = glob.glob('images/cap0-*.png')
imgPaths2 = glob.glob('images/cap2-*.png')

for i in range(len(imgPaths1)):
    boardCorners1 = getCornersFromImageWithPath(imgPaths1[i], board, dictionary)
    boardCorners2 = getCornersFromImageWithPath(imgPaths2[i], board, dictionary)

    wholeBoardFound1 = (boardCorners1 is not None and len(boardCorners1) == numCorners)
    wholeBoardFound2 = (boardCorners2 is not None and len(boardCorners2) == numCorners)

    if wholeBoardFound1 and wholeBoardFound2:
        allCorners1.append(boardCorners1)
        allCorners2.append(boardCorners2)

# get image size from a single image
img = cv2.imread('images/cap0-0.png')
height,width,_ = img.shape

objectCorners = board.chessboardCorners
allObjectCorners = np.repeat(objectCorners[np.newaxis,:,:], len(allCorners1), axis=0)

_,_,_,_,_,R,T,_,_ = cv2.stereoCalibrate(allObjectCorners, allCorners1, allCorners2, mtx1, dist1, mtx2, dist2, (height,width), flags=cv2.CALIB_FIX_INTRINSIC)
R1,R2,P1,P2,_,_,_ = cv2.stereoRectify(mtx1, dist1, mtx2, dist2, (height, width), R, T)

np.savez('calib-stereo',R=R,T=T,R1=R1,R2=R2,P1=P1,P2=P2)

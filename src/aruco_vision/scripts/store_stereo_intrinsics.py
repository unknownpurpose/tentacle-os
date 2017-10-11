#!/usr/bin/env python

import numpy as np
import cv2
import glob

# generate board
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(5,7,.025,.0125,dictionary)

# calculate intrinsics
#   for each image, find markers then interpolate chessboard corners from markers.
#   create uber-array of all ids and found corners (for all images)

def getIntrinsicsFromImages(imgPaths):
    corners = []
    ids = []
    img = None # keep a ref to the last image just so we can get its size later

    for imgPath in imgPaths:
        img = cv2.imread(imgPath)
        mCorners, mIds, _ = cv2.aruco.detectMarkers(img, dictionary)
        if mIds is not None:
            _, boardCorners, boardIds = cv2.aruco.interpolateCornersCharuco(mCorners, mIds, img, board)
            if boardCorners is not None and len(boardCorners) >= 4:
                corners.append(boardCorners)
                ids.append(boardIds)

    _, cm, dc, _, _ = cv2.aruco.calibrateCameraCharuco(corners, ids, board, img.shape[0:2], None, None)
    return cm, dc

cm0, dc0 = getIntrinsicsFromImages(glob.glob('images/cap0-*.png'))
cm2, dc2 = getIntrinsicsFromImages(glob.glob('images/cap2-*.png'))

# store in a calibration file for later
np.savez('calib.npz',mtx1=cm0,dist1=dc0,mtx2=cm2,dist2=dc2)

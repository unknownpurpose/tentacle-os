#!/usr/bin/env python

import cv2

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(5,7,.025,.0125,dictionary)
img = board.draw((300*5,300*7))

#Dump the calibration board to a file
cv2.imwrite('charuco.png',img)

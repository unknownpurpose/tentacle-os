#!/usr/bin/env python

import cv2
import numpy as np
import time

from helpers import percentVisible

print('Firin uppp')

numPics = 12
capNum = 0
lastcapture = 0
interval = 3.0 # take 3 secs between each capture
threshold = 0.5 # min % of visible IDs for capture

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(5,7,.025,.0125,dictionary)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"H264"))
cap2 = cv2.VideoCapture(1)
cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"H264"))

while True:
    _,frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _,frame = cap2.read()
    gray2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    now = time.time()
    if (now - lastcapture) > interval:
        pVis = percentVisible(gray, gray2, board, dictionary)
        print('{:.0f}% visible'.format(pVis*100))

        if pVis >= threshold:
            print('Capture!')
            cv2.imwrite('images/cap0-{}.png'.format(capNum), gray)
            cv2.imwrite('images/cap2-{}.png'.format(capNum), gray2)

            lastcapture = now
            capNum += 1

            if (capNum == numPics):
                break

cap.release()
cap2.release()

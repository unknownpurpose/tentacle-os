#!/usr/bin/env python

import cv2
import numpy as np
import time

print('Firin uppp')

numPics = 20
capNum = 0
lastcapture = 0
debounce = 2.0

cap = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(2)

while True:
    _,frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _,frame = cap2.read()
    gray2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    vis = np.concatenate((gray, gray2), axis=1) # combine horiz
    vis = cv2.resize(vis, (960,270))
    cv2.imshow('frame', vis)

    now = time.time()
    if cv2.waitKey(1) == ord('a') and (now - lastcapture) > debounce:
        print('grab!\n')
        cv2.imwrite('images/cap0-{}.png'.format(capNum), gray)
        cv2.imwrite('images/cap2-{}.png'.format(capNum), gray2)

        lastcapture = now
        capNum += 1

        if (capNum == numPics):
            break

cap.release()
cap2.release()
cv2.destroyAllWindows()

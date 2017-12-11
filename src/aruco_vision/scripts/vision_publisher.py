#!/usr/bin/env python

import rospy
import numpy as np
import transforms3d.quaternions as qs
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
# from cv_bridge.CvBridge import cv2_to_imgmsg

capL,capR = None, None

def get_position_from_points(points):
    points /= points[3] # NORMALISE!
    points = points[0:3].T # flip it to Nx3
    p = np.mean(points, axis=0) # average points to get 1x3 matrix in the centre
    position = Point(x=p[0],y=p[1],z=p[2])
    return position

def start():
    rospy.init_node('vision_publisher', anonymous=True)
    position_publisher = rospy.Publisher('aruco/position', Point, queue_size=10)
    image_publisher_left = rospy.Publisher('camera/left', Image, queue_size=10)
    image_publisher_right = rospy.Publisher('camera/right', Image, queue_size=10)

    bridge = CvBridge()

    calib = np.load('calibs/calib.npz')
    calibStereo = np.load('calibs/calib-stereo.npz')

    mtx1,dist1 = calib['mtx1'],calib['dist1']
    mtx2,dist2 = calib['mtx2'],calib['dist2']

    R1,P1 = calibStereo['R1'],calibStereo['P1']
    R2,P2 = calibStereo['R2'],calibStereo['P2']

    capL = cv2.VideoCapture(0)
    capL.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"H264"))
    capL.set(cv2.CAP_PROP_FPS, 10.0)
    capR = cv2.VideoCapture(1)
    capR.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"H264"))
    capR.set(cv2.CAP_PROP_FPS, 10.0)

    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters =  cv2.aruco.DetectorParameters_create()
    markerLength = 39 # millimetres

    while not rospy.is_shutdown():
        _,frameL = capL.read()
        _,frameR = capR.read()

        greyL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
        greyR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)

        msgL = bridge.cv2_to_imgmsg(greyL, encoding="passthrough")
        image_publisher_left.publish(msgL)
        msgR = bridge.cv2_to_imgmsg(greyR, encoding="passthrough")
        image_publisher_right.publish(msgR)

        cornersL, idsL, _ = cv2.aruco.detectMarkers(greyL, aruco_dict, parameters=parameters)
        cornersR, idsR, _ = cv2.aruco.detectMarkers(greyR, aruco_dict, parameters=parameters)

        if (idsL is None) or (idsR is None): continue # ID not seen by both cameras
        foundIds = np.intersect1d(idsL, idsR)
        if (len(foundIds) == 0): continue # IDs found, but not the same one

        m = foundIds[0] # we take the first found ID

        print('Found id {}.'.format(m))

        cL = cornersL [ np.where(idsL == m)[0] ][0] # get the 2x4 set of corners matching the found ID
        cR = cornersR [ np.where(idsR == m)[0] ][0] # get the 2x4 set of corners matching the found ID

        if len(cornersL) > 0:
            points = cv2.triangulatePoints(P1, P2, cL.T, cR.T)
            position = get_position_from_points(points)
            position_publisher.publish(position)

def shutdown():
    capL.release()
    capR.release()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        shutdown()
        pass

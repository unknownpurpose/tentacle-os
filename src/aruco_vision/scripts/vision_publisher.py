#!/usr/bin/env python

import rospy
import numpy as np
import transforms3d.quaternions as qs
from geometry_msgs.msg import Quaternion, Pose, Point
import cv2

capL,capR = None, None

def get_single_marker(cornersL, cornersR, idsL, idsR):
    foundIds = np.intersect1d(idsL, idsR)

    corners = corners1[np.in1d(ids1,foundIds)]

def get_pose_from_rotation_translation(rvec, tvec):
    pose_pt = Point(x=tvec[0],y=tvec[1],z=tvec[2])
    rm = cv2.Rodrigues(rvec) # calculate rotation matrix from rvec
    qt = qs.mat2quat(rm) # turn it into a quaternion
    pose_qt = Quaternion(x=qt[0],y=qt[1],z=qt[2],w=qt[3])
    pose = Pose(position=pose_pt, orientation=pose_qt)
    return pose

def get_position_from_points(points):
    points /= p[3] # NORMALISE!
    points = points[0:2].T # flip it to Nx3
    p = np.mean(points, axis=0) # average points to get 1x3 matrix in the centre

    position = Point(x=p[0],y=p[1],z=p[2])
    return position

def start():
    rospy.init_node('pose_publisher', anonymous=True)
    pose_publisher = rospy.Publisher('aruco/pose', Pose, queue_size=10)
    position_publisher = rospy.Publisher('aruco/position', Quaternion, queue_size=10)

    calib = np.load('calibs/calib.npz')
    calibStereo = np.load('calibs/calib-stereo.npz')

    mtx1,dist1 = calib['mtx1'],calib['dist1']
    mtx2,dist2 = calib['mtx2'],calib['dist2']

    R1,P1 = calibStereo['R1'],calibStereo['P1']
    R2,P2 = calibStereo['R2'],calibStereo['P2']

    capL = cv2.VideoCapture(0)
    capL.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"H264"))
    capR = cv2.VideoCapture(1)
    capR.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"H264"))

    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters =  cv2.aruco.DetectorParameters_create()
    markerLength = 39 # millimetres

    while not rospy.is_shutdown():
        _,frameL = capL.read()
        _,frameR = capR.read()

        greyL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
        greyR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)

        cornersL, idsL, _ = cv2.aruco.detectMarkers(greyL, aruco_dict, parameters=parameters)
        cornersR, idsR, _ = cv2.aruco.detectMarkers(greyR, aruco_dict, parameters=parameters)

        if (idsL is None) or (idsR is None): continue
        foundIds = np.intersect1d(idsL, idsR)
        if (len(foundIds) == 0): continue
        m = foundIds[0] # we take the first found ID

        cL = cornersL [ np.where(idsL == m)[0] ] # get the set of corners matching the found ID
        cR = cornersR [ np.where(idsR == m)[0] ] # get the set of corners matching the found ID

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(cL, markerLength, mtx1, dist1)
        if rvecs is not None:
            print('marker spotted')
            rvec,tvec = rvecs[0][0],tvecs[0][0]

            #pose = get_pose_from_rotation_translation(rvec, tvec)
            #pose_publisher.publish(pose)

        if len(cornersL) > 0:
            print(np.array(idsL))
            points = cv2.triangulatePoints(P1, P2, cornersL, cornersR)
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

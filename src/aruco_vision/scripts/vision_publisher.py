#!/usr/bin/env python

import rospy
import numpy as np
import transforms3d.quaternions as qs
from geometry_msgs.msg import Quaternion, Pose
import cv2

capL,capR = None

def start():
    rospy.init_node('vision_publisher', anonymous=True)
    pose_publisher = rospy.Publisher('aruco/pose', Pose, queue_size=10)
    position_publisher = rospy.Publisher('aruco/position', Quaternion, queue_size=10)

    calib = np.load('calib.npz')
    calibStereo = np.load('calib-stereo.npz')

    mtx1,dist1 = calib['mtx1'],calib['dist1']
    mtx2,dist2 = calib['mtx2'],calib['dist2']

    R1,P1 = calibStereo['R1'],calibStereo['P1']
    R2,P2 = calibStereo['R2'],calibStereo['P2']

    capL = cv2.VideoCapture(0)
    capR = cv2.VideoCapture(2)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters =  aruco.DetectorParameters_create()
    markerLength = 39 # millimetres

    # _,img = capL.read()
    # height,width,_ = img.shape

    while not rospy.is_shutdown():
        _,frameL = capL.read()
        _,frameR = capR.read()

        greyL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
        greyR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)

        cornersL, idsL, _ = aruco.detectMarkers(greyL, aruco_dict, parameters=parameters)
        cornersR, idsR, _ = aruco.detectMarkers(greyR, aruco_dict, parameters=parameters)

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, dist)
        rvec,tvec = rvecs[0],tvecs[0] # only interested in our single marker

        pose_pt = Point(x=tvec.x,y=tvec.y,z=tvec.z)
        rm = cv2.Rodrigues(rvec) # calculate rotation matrix from rvec
        qt = sq.mat2quat(rm) # turn it into a quaternion
        pose_qt = Quaternion(x=qt[0],y=qt[1],z=qt[2],w=qt[3])
        pose = Pose(position=pose_pt, orientation=pose_qt)
        pose_publisher.publish(pose)

        points = cv2.triangulatePoints(P1, P2, cornersL, cornersR)
        points /= p[3] # NORMALISE!
        points = points[0:2].T # flip it to Nx3
        p = np.mean(points, axis=0) # average points to get 1x3 matrix in the centre
        position = Point(x=p[0],y=p[1],z=p[2])
        position_publisher.publish(position)

        vis = np.concatenate((greyL, greyR), axis=1) # combine horiz
        vis = cv2.resize(vis, (960,270))
        cv2.imshow('frame', vis)

        if cv2.waitKey(1) == ord('q'):
            break

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

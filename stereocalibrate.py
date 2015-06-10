#!/usr/bin/env python
#
# OpenCV calibration example with changes by Emaneule Ruffaldi SSSA 2015
# Improved argument mechanism, exposed parameters
import numpy as np
import cv2,yaml,os
import sys
import argparse
from glob import glob

def splitfn(x):
    import os
    path,name = os.path.split(x)
    name,ext = os.path.splitext(x)
    return path,name,ext

def loadcalib(name):
    o = yaml.load(open(name,"rb"))
    o["camera_matrix"] = np.array(o["camera_matrix"],dtype=np.float32)
    o["dist"]= np.array(o["dist"],dtype=np.float32) 
    return o

def loadimageinfo(name):
    o = open(name,"rb")
    d = yaml.load(o)
    d["image_points"] = np.array(d["image_points"],dtype=np.float32)
    d["world_points"] = np.array(d["world_points"],dtype=np.float32)
    return d

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Stereo Pair Calibrator - OpenCV and Emanuele Ruffaldi SSSA 2014-2015')
    parser.add_argument('--images1', help='path to yaml files of camera 1',required=True)
    parser.add_argument('--images2', help='path to yaml files of camera 2',required=True)
    parser.add_argument('--calib1', help='calibration file of camera 1',required=True)
    parser.add_argument('--calib2', help='calibration file of camera 2',required=True)
    parser.add_argument('--save', help='name of output calibration in YAML otherwise prints on console')
    parser.add_argument('--verbose',action="store_true")
    args = parser.parse_args()

    term_crit = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 200, 1e-8)

    images = [glob(args.images1+"/*.yaml"),glob(args.images2+"/*.yaml")]
    images2set = dict()
    for x in images[1]:
        images2set[os.path.split(x)[1]] = x


    paramir = args.calib1
    paramrgb = args.calib2

    cam1calib = loadcalib(paramir)
    cam2calib = loadcalib(paramrgb)
    print "input CAM1 calib",cam1calib
    print "input CAM2 calib",cam2calib

    obj_points = []
    img_points1 = []
    img_points2 = []
    #    if obj_points is None:
    #        objp = np.zeros((14*14,3), np.float32)
    #        objp[:,:2] = np.mgrid[0:14,0:14].T.reshape(-1,2)        

    for i,e in enumerate(images[0]):            
        fn  = os.path.split(e)[1]
        if not fn in images2set:
            continue
        cam1info = loadimageinfo(e)
        cam2info = loadimageinfo(images2set[fn])
        cam1size = (cam1info["height"],cam1info["width"])
        cam2size = (cam2info["height"],cam2info["width"])
        print fn,cam1size,cam2size,cam1info["world_points"].shape,cam1info["image_points"].shape,cam2info["image_points"].shape
        obj_points.append(cam1info["world_points"])
        img_points1.append(cam1info["image_points"])
        img_points2.append(cam2info["image_points"])


    flags = 0
    flags |= cv2.CALIB_FIX_INTRINSIC
    #flags |= cv2.CALIB_USE_INTRINSIC_GUESS
    #flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    #flags |= cv2.CALIB_FIX_FOCAL_LENGTH
    #flags |= cv2.CALIB_FIX_ASPECT_RATIO
    #flags |= cv2.CALIB_ZERO_TANGENT_DIST
    #flags |= cv2.CALIB_SAME_FOCAL_LENGTH
    #flags |= cv2.CALIB_RATIONAL_MODEL
    flags |= cv2.CALIB_FIX_K3
    flags |= cv2.CALIB_FIX_K4
    flags |= cv2.CALIB_FIX_K5
    retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F  = cv2.stereoCalibrate(obj_points,img_points1,img_points2,cam1size,cam1calib["camera_matrix"],cam1calib["dist"],cam2calib["camera_matrix"],cam2calib["dist"],criteria=term_crit,flags=flags)

    print "error is:",retval
    print "orig cameraMatrix1\n",cam1calib["camera_matrix"]
    print "cameraMatrix1\n",cameraMatrix1
    print "orig cameraMatrix2\n",cam2calib["camera_matrix"]
    print "cameraMatrix2\n",cameraMatrix2
    print "orig distCoeffs1",cam1calib["dist"]
    print "distCoeffs1",distCoeffs1.transpose()
    print "orig distCoeffs1",cam2calib["dist"]
    print "distCoeffs1",distCoeffs2.transpose()
    print "T\n",T
    print "R\n",R
    
    #cv2.destroyAllWindows()

    outname = args.save
    if outname is not None:
        ci = dict(rms=retval,T=T.tolist(),R=R.tolist(),E=E.tolist(),F=F.tolist(),K1=cameraMatrix1.tolist(),K2=cameraMatrix2.tolist(),d1=distCoeffs1.tolist(),d2=distCoeffs2.tolist())
        print ci
        yaml.dump(ci,open(outname,"wb"))

#!/usr/bin/env python
#
# OpenCV calibration example with changes by Emaneule Ruffaldi SSSA 2015
# Improved argument mechanism, exposed parameters
import numpy as np
import cv2,yaml,os
import sys
import argparse
from glob import glob
from se3d import * 

def recaberror(poses1,poses2,tf2to1):
    return  math.sqrt(sum([se3_dist(se3_mul(tf2to1,poses2[x]),poses1[x]) for x in range(0,len(poses1))]))/len(poses1)


yamltemplate = """%YAML:1.0
rgb_intrinsics: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [$RGBK$]
rgb_distortion: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [$RGBD$]
depth_intrinsics: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [$DEPTHK$]
depth_distortion: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [$DEPTHD$]
R: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ $R$]
T: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [ $T$]
R_extrinsics: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [$R$]
T_extrinsics: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [ $T$]
rgb_size: !!opencv-matrix
   rows: 1
   cols: 2
   dt: i
   data: [ $RGBSIZE$]
raw_rgb_size: !!opencv-matrix
   rows: 1
   cols: 2
   dt: i
   data: [ $RGBSIZE$]
depth_size: !!opencv-matrix
   rows: 1
   cols: 2
   dt: i
   data: [ $DEPTHSIZE$]
raw_depth_size: !!opencv-matrix
   rows: 1
   cols: 2
   dt: i
   data: [ $DEPTHSIZE$]
raw_depth_unit_in_meters: !!opencv-matrix
   rows: 1
   cols: 1
   dt: f
   data: [ 1.00000005e-003 ]
min_max_depth_in_meters: !!opencv-matrix
   rows: 2
   cols: 1
   dt: f
   data: [ 4.00000006e-001, 5. ]
infrared_size: !!opencv-matrix
   rows: 1
   cols: 2
   dt: i
   data: [ 0, 0 ]
depth_base_and_offset: !!opencv-matrix
   rows: 1
   cols: 2
   dt: f
   data: [ 7.50000030e-002, 1090. ]
depth_multiplicative_correction_factor: !!opencv-matrix
   rows: 1
   cols: 1
   dt: f
   data: [ 1. ]
depth_additive_correction_factor: !!opencv-matrix
   rows: 1
   cols: 1
   dt: f
   data: [ 0. ]"""


def flatten(y):
    return ",".join([str(x) for x in y.flatten()])
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

    parser = argparse.ArgumentParser(description='Stereo Pair Calibrator (SE3 verison) - OpenCV and Emanuele Ruffaldi SSSA 2014-2015')
    parser.add_argument('--images1', help='path to yaml files of camera 1',required=True)
    parser.add_argument('--images2', help='path to yaml files of camera 2',required=True)
    parser.add_argument('--calib1', help='calibration file of camera 1',required=True)
    parser.add_argument('--calib2', help='calibration file of camera 2',required=True)
    parser.add_argument('--suffix1', help='suffix for key',default="")
    parser.add_argument('--suffix2', help='suffix for key',default="")
    parser.add_argument('--savealt', help='name of output calibration in YAML otherwise prints on console')
    parser.add_argument('--save', help='name of output calibration in YAML otherwise prints on console')
    parser.add_argument('--verbose',action="store_true")
    args = parser.parse_args()

    term_crit = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 1000, 1e-8)

    images = []
    if os.path.isdir(args.images1):
        images.append(glob(args.images1+"/*.yaml"))
    else:
        images.append(glob(args.images1))
    if os.path.isdir(args.images2):
        images.append(glob(args.images2+"/*.yaml"))
    else:
        images.append(glob(args.images2))
    images2set = dict()
    for x in images[1]:
        images2set[os.path.split(x)[1][0:-len(args.suffix2)]] = x  # remove the suffix if any


    param1 = args.calib1
    param2 = args.calib2

    cam1calib = loadcalib(param1)
    cam2calib = loadcalib(param2)
    print "input CAM1 calib",cam1calib
    print "input CAM2 calib",cam2calib

    poses = []
    poses1 = []
    poses2 = []
    relposes = []
    img_points1 = [] # for reprojection
    img_points2 = [] # for reprojection
    #    if obj_points is None:
    #        objp = np.zeros((14*14,3), np.float32)
    #        objp[:,:2] = np.mgrid[0:14,0:14].T.reshape(-1,2)        

    for i,e in enumerate(images[0]):            
        fn  = os.path.split(e)[1]
        fn = fn[0:-len(args.suffix1)] # remove the suffix if any
        if not fn in images2set:
            print "missing file in set 1",e
            continue
        cam1info = loadimageinfo(e)
        if len(cam1info["image_points"]) == 0:
            print "missing chessboard in set 1 for file",e
            continue
        cam2info = loadimageinfo(images2set[fn])
        if len(cam2info["image_points"]) == 0:
            print "missing chessboard in set 2 for file",images2set[fn]
        else:
            cam1size = (cam1info["height"],cam1info["width"])
            cam2size = (cam2info["height"],cam2info["width"])
            print fn,cam1size,cam2size,cam1info["world_points"].shape,cam1info["image_points"].shape,cam2info["image_points"].shape
            #obj_points.append(cam1info["world_points"])
            if  "rvec" in cam1info and "rvec" in cam2info:
              poses.append(cam1info["rvec"] + cam1info["tvec"] + cam2info["rvec"] + cam2info["tvec"])
              img_points1.append(cam1info["image_points"])
              img_points2.append(cam2info["image_points"])
              p1 = se3_fromRvecT(cam1info["rvec"],cam1info["tvec"])   # O -> P1
              p2 = se3_fromRvecT(cam2info["rvec"],cam2info["tvec"])   # O -> P2
              p2to1 = se3_mul(se3_inv(p2),p1)      # P2 -> P1 = inv(O -> P2) (O->P1)
              relposes.append(p2to1)
              poses1.append(p1)
              poses2.append(p2)

    f = open("out.dat","wb")
    for x in poses:
      f.write(",".join([str(y[0]) for y in x]))
      f.write("\n")
    print "Done ",len(poses)
    f.close()
    #retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F  = cv2.stereoCalibrate(obj_points,img_points1,img_points2,cam1size,cam1calib["camera_matrix"],cam1calib["dist"],cam2calib["camera_matrix"],cam2calib["dist"],criteria=term_crit,flags=flags)
    #compute poses from SE3 tool, then reproject
    se3d =  se3d_est(relposes,10)
    T = se3_gett(se3d["mean"])
    R = se3_getR(se3d["mean"])
    r = se3_getrvec(se3d["mean"])
    cameraMatrix1 = cam1calib["camera_matrix"]
    cameraMatrix2 = cam2calib["camera_matrix"]
    distCoeffs1 = cam1calib["dist"]
    distCoeffs2 = cam1calib["dist"]
    # TODO reproject
    retval = recaberror(poses1,poses2,se3d["mean"])
    print "error is:",retval
    print "orig cameraMatrix1\n",cameraMatrix1
    print "orig cameraMatrix2\n",cameraMatrix2
    print "orig distCoeffs1",cam1calib["dist"]
    print "orig distCoeffs1",cam2calib["dist"]
    print "T\n",T
    print "R\n",R
    print "rvec\n",r
    print "S\n",se3d["Sigma"]

    outname = args.savealt
    if outname is not None:
        ci = dict(rms=retval,T=T.tolist(),R=R.tolist(),K1=cameraMatrix1.tolist(),K2=cameraMatrix2.tolist(),d1=distCoeffs1.tolist(),d2=distCoeffs2.tolist())
        print ci
        yaml.dump(ci,open(outname,"wb"))

    outname = args.save
    if outname is not None:
        # Save 
        d = {}
        d["$RGBK$"] = flatten(cameraMatrix1)
        d["$RGBD$"] = flatten(distCoeffs1[0:5])
        d["$DEPTHK$"] = flatten(cameraMatrix2)
        d["$DEPTHD$"] = flatten(distCoeffs2[0:5])
        d["$R$"] = flatten(R)
        d["$T$"] = flatten(T)

        d["$RGBSIZE$"] = "%d,%d" % (cam1size[1],cam1size[0])
        d["$DEPTHSIZE$"] = "%d,%d" % (cam2size[1],cam2size[0])
        yamlout = yamltemplate
        for k,v in d.iteritems():
            yamlout = yamlout.replace(k,v)
        open(outname,"wb").write(yamlout)


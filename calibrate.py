#!/usr/bin/env python
#
# OpenCV calibration example with changes by Emaneule Ruffaldi SSSA 2015
# - Improved argument mechanism, exposed parameters
# - Match by filename not order
#
#
# TODO: use camera_info from rostopic echo
# TODO: fix intrinsics (only mapping)
# TODO: don't recompute chessboard
import numpy as np
import os,cv2,yaml,argparse
from glob import glob

def splitfn(x):
    path,name = os.path.split(x)
    name,ext = os.path.splitext(x)
    return path,name,ext

def coords(s,t):
    try:
        x, y = map(int, s.split(','))
        return x, y
    except:
        raise argparse.ArgumentTypeError("%s must be x,y" % t)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Camera Calibrator - OpenCV and Emanuele Ruffaldi SSSA 2014-2015')
    parser.add_argument('path', help='path where images can be found (png or jpg)',nargs="+")
    parser.add_argument('--save', help='name of output calibration in YAML otherwise prints on console')
    parser.add_argument('--verbose',action="store_true")
    parser.add_argument('--ir',action='store_true')
    #parser.add_argument('--load',help="read intrinsics from file")
    #parser.add_argument('--nocalibrate',action="store_true",help="performs only reprojection")
    #parser.add_argument('--noextract',action="store_true",help="assumes features already computed (using yaml files and not the images)")
    parser.add_argument('--debug',help="debug dir for chessboard markers",default="")
    parser.add_argument('--pattern_size',default=(6,9),help="pattern as (w,h)",type=lambda s: coords(s,'Pattern'), nargs=2)
    parser.add_argument('--target_size',default=None,help="target image as (w,h) pixels",type=lambda s: coords(s,'Target Image'), nargs=2)
    parser.add_argument('--aperture',default=None,help="sensor size in m as (w,h)",type=lambda s: coords(s,'Aperture'), nargs=2)
    parser.add_argument('--square_size',help='square size in m',type=float,default=0.025)
    parser.add_argument('--nodistortion',action="store_true");
    args = parser.parse_args()


    if args.ir:
        eflags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    else:
        eflags = cv2.CALIB_CB_ADAPTIVE_THRESH
    if False:
        if args.intrinsics != None:
            # load yaml
            pass
        if args.nocalibrate:
            pass
        if args.noextract:
            pass

    img_names = []
    for p in args.path:
        img_names.extend(glob(p))
    debug_dir = args.debug
    square_size = args.square_size

    pattern_size_cols_rows = (args.pattern_size[0],args.pattern_size[1])
    pattern_points = np.zeros( (np.prod(pattern_size_cols_rows), 3), np.float32 )
    pattern_points[:,:2] = np.indices(pattern_size_cols_rows).T.reshape(-1, 2)
    pattern_points *= args.square_size

    obj_points = []
    img_points = []
    target = args.target_size
    h, w = 0, 0
    lastsize = None
    criteriasub = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    criteriacal = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 120, 0.001)

    #giacomo
    #for both sub and cal cv::TermCriteria term_criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON);

    print "images",img_names
    for fn in sorted(img_names):
        print fn,'processing'
        img = cv2.imread(fn, 0)
        if img is None:
          print fn,"failed to load"
          continue
        h, w = img.shape[:2]
        if target is not None and (h,w) != target:
            print fn, (h,w),"->",target
            img = cv2.resize(img,target)
            h,w = target
        else:
            if lastsize is None:
                lastsize = (h,w)
                print "using",(h,w)
            else:
                if lastsize != (h,w):
                    print fn, "all images should be the same size, enforcing"
                    target = lastsize
                    img = cv2.resize(img,target)
                    h,w = target
        found, corners = cv2.findChessboardCorners(img, pattern_size_cols_rows,flags=eflags)
        if found:
            # Giacomo (11,11)
            cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), criteriasub)
        if debug_dir == "":
            vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(vis, pattern_size_cols_rows, corners, found)
            path, name, ext = splitfn(fn)
            cv2.imwrite('%s/%s_chess.png' % (debug_dir, name), vis)
        if not found:
            print fn,'chessboard not found'
            continue
        info = dict(width=w,height=h,image_points=corners.reshape(-1,2).tolist(),world_points=pattern_points.tolist())
        yaml.dump(info,open(os.path.splitext(fn)[0]+".yaml","wb"))
        img_points.append(corners.reshape(-1, 2))
        obj_points.append(pattern_points)

    
    #CV_CALIB_USE_INTRINSIC_GUESS
    
    flags = 0
    if args.nodistortion:
        flags = cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6 | cv2.CALIB_ZERO_TANGENT_DIST
    print "calibrating..."
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None,criteria=criteriacal,flags=flags)
    print "error:", rms
    print "camera matrix:\n", camera_matrix
    print "distortion coefficients:", dist_coefs.transpose()
    #cv2.destroyAllWindows()

    #apertureWidth
    #apertureHeight
    if args.aperture:
        fovx,fovy,focalLength,principalPoint,aspectRatio = cv2.calibrationMatrixValues(camera_matrix,(w,h),args.aperture[0],args.aperture[1])

    if False:
        # This is the code that computes rms
        tot_error = 0
        for i in xrange(len(obj_points)):
            imgpoints2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coefs)
            error = cv2.norm(img_points[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            tot_error += error
        mean_error = tot_error/len(obj_points)
        print "total error: ", mean_error

    outname = args.save
    if outname is not None:
        ci = dict(image_width=w,image_height=h,pattern_size=list(pattern_size_cols_rows),rms=rms,camera_matrix=camera_matrix.tolist(),dist=dist_coefs.ravel().tolist(),square_size=square_size)
        print ci
        yaml.dump(ci,open(outname,"wb"))

#!/usr/bin/env python
#
# OpenCV calibration example with changes by Emaneule Ruffaldi SSSA 2015
# - Improved argument mechanism, exposed parameters
# - Match by filename not order
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
    parser.add_argument('--debug',help="debug dir for chessboard markers",default="")
    parser.add_argument('--pattern_size',default=(9,6),help="pattern as (w,h)",type=lambda s: coords(s,'Pattern'), nargs=2)
    parser.add_argument('--target_size',default=None,help="target image as (w,h) pixels",type=lambda s: coords(s,'Target Image'), nargs=2)
    parser.add_argument('--aperture',default=None,help="sensor size in m as (w,h)",type=lambda s: coords(s,'Aperture'), nargs=2)
    parser.add_argument('--square_size',help='square size in m',type=float,default=0.025)
    args = parser.parse_args()

    img_names = []
    for p in args.path:
        img_names.extend(glob(p))
    debug_dir = args.debug
    square_size = args.square_size

    pattern_size = args.pattern_size
    pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
    pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= args.square_size

    obj_points = []
    img_points = []
    target = args.target_size
    h, w = 0, 0
    lastsize = None
    criteriasub = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    criteriacal = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 120, 0.001)

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
        found, corners = cv2.findChessboardCorners(img, pattern_size)
        if found:
            cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), criteriasub)
        if debug_dir == "":
            vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(vis, pattern_size, corners, found)
            path, name, ext = splitfn(fn)
            cv2.imwrite('%s/%s_chess.png' % (debug_dir, name), vis)
        if not found:
            print fn,'chessboard not found'
            continue
        info = dict(width=w,height=h,image_points=corners.reshape(-1,2).tolist(),world_points=pattern_points.tolist())
        yaml.dump(info,open(os.path.splitext(fn)[0]+".yaml","wb"))
        img_points.append(corners.reshape(-1, 2))
        obj_points.append(pattern_points)


    print "calibrating..."
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None,criteria=criteriacal)
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
        ci = dict(image_width=w,image_height=h,pattern_size=list(pattern_size),rms=rms,camera_matrix=camera_matrix.tolist(),dist=dist_coefs.ravel().tolist(),square_size=square_size)
        print ci
        yaml.dump(ci,open(outname,"wb"))

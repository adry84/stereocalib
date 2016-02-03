
import numpy as np
import os,cv2,yaml,argparse
from glob import glob
import sys


def main():
    src1 = sys.argv[1]
    src2 = sys.argv[2]
    dst = sys.argv[3]

    img_names = []
    img_names.extend(glob(src1+"/*"))

    for f1 in img_names:
        p1,n1 = os.path.split(f1)
        f2 = os.path.join(src2,n1)
        fd = os.path.join(dst,n1)

        if not os.path.isfile(f2):
            print "missing",f1,n1,f2
        else:
            img1 = cv2.imread(f1, 0)
            img2 = cv2.imread(f2, 0)
            d = cv2.absdiff(img1,img2)

            cv2.imwrite(fd,d)
           
    print "len",len(img_names)
if __name__ == '__main__':
    main()


import cv2
import numpy as np
import sys
filename = sys.argv[1]
img = cv2.imread(filename)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

gray = np.float32(gray)
th, dst = cv2.threshold(gray, 230, 255, cv2.THRESH_BINARY);
#dst = cv2.cornerHarris(gray,int(sys.argv[2]),int(sys.argv[3]),0.04)

#result is dilated for marking the corners, not important
#dst = cv2.dilate(dst,None)

# Threshold for an optimal value, it may vary depending on the image.
#img[dst>0.01*dst.max()]=[0,0,255]

cv2.imshow('dst',dst)
if cv2.waitKey(0) & 0xff == 27:
    cv2.destroyAllWindows()
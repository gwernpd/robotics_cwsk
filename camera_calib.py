import cv2
import numpy as np
import glob

# Load calibration images
img = cv2.imread("camera.jpg")
#img = np.array(img)


# Color-segmentation to get binary mask
lwr = np.array([0, 0, 143])
upr = np.array([179, 61, 252])
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
msk = cv2.inRange(hsv, lwr, upr)

# Extract chess-board
krn = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 30))
dlt = cv2.dilate(msk, krn, iterations=5)
res = 255 - cv2.bitwise_and(dlt, msk)

# Displaying chess-board features
res = np.uint8(res)
ret, corners = cv2.findChessboardCorners(res, (7, 5),
                                         flags=cv2.CALIB_CB_ADAPTIVE_THRESH +
                                               cv2.CALIB_CB_FAST_CHECK +
                                               cv2.CALIB_CB_NORMALIZE_IMAGE)
if ret:
    print(corners)
    fnl = cv2.drawChessboardCorners(img, (7, 5), corners, ret)
    cv2.imshow("fnl", fnl)
    cv2.waitKey(0)
else:
    print("No Checkerboard Found")
    cv2.imshow("fnl", res)
    cv2.waitKey(0)
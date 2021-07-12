import numpy as np
import cv2
import imutils
from matplotlib import pyplot as plt


path = '/home/iftach/opencv_tutorials/coins/coins.png'
img = cv2.imread(path)
grayScaleImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
bluringImg = cv2.GaussianBlur(grayScaleImg,(5,5), cv2.BORDER_DEFAULT)
retGray, treshholdGray = cv2.threshold(grayScaleImg, 220, 255, cv2.THRESH_BINARY_INV)
retBlur, threshholdBlur = cv2.threshold(bluringImg, 220, 255, cv2.THRESH_BINARY_INV)

kernel = np.ones((3,3), np.uint8)
opening = cv2.morphologyEx(threshholdBlur, cv2.MORPH_OPEN, kernel)
dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
ret, last_image = cv2.threshold(dist_transform, 0.3*dist_transform.max(), 255, 0)
last_image = np.uint8(last_image)

cnts = cv2.findContours(last_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)

for (i, c) in enumerate(cnts):
    ((x,y), _) = cv2.minEnclosingCircle(c)
    cv2.putText(img, "#{}".format(i + 1), (int(x) - 50, int(y)+50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 1)  
    cv2.drawContours(img, [c], -1, (255,0,0), 2)
cv2.putText(img, "Total of {} Coins".format(i), (150, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)


cv2.imshow('Original Image',img)

cv2.waitKey(0) 
cv2.destroyAllWindows() 
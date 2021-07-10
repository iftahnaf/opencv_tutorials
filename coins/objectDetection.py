import numpy as np
import cv2
from matplotlib import pyplot as plt


path = '/home/iftach/opencv_tutorials/coins/coins.png'
img = cv2.imread(path)
grayScaleImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, treshhold = cv2.threshold(grayScaleImg,240,255,cv2.THRESH_BINARY_INV)

cv2.imshow('Original Image',img)
cv2.imshow('Gray Scaled Image', grayScaleImg)
cv2.imshow('Treshold Image', treshhold)

cv2.waitKey(0) 
cv2.destroyAllWindows() 
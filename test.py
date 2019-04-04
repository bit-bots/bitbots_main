#!/usr/bin/env python2
import numpy
import cv2

def show(image):
    cv2.imshow('image',image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def feature_value(image):
    mask = cv2.Canny(image,100,200)
    show(mask)
    wert = float(cv2.countNonZero(mask))/mask.size
    return wert


img = cv2.imread('frame3.png')
img1 = cv2.imread('frame4.png')

print((feature_value(img), feature_value(img1)))



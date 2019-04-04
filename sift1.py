#!/usr/bin/env python2
import numpy
import cv2
import silx
from silx.image import sift

def show(image):
    cv2.imshow('image',image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def compare(image1, image2):
    sift_ocl = sift.SiftPlan(template=image1, devicetype="CPU")
    #print("Device used for calculation: ", sift_ocl.ctx.devices[0].name)

    kp = sift_ocl(image1)
    kp1 = sift_ocl(image2)

    mp = sift.MatchPlan()
    match = mp(kp, kp1)

    # (Anzahl Keypoints Image1, Anzahl Keypoints Image2, Matched Keypoints)
    return (len(kp), len(kp1), match.shape[0])

imgs = [cv2.imread('frame1.png'),
        cv2.imread('frame2.png'),
        cv2.imread('frame3.png'),
        cv2.imread('frame4.png'),]

print("Frame 1 + 2: {}".format(compare(imgs[0], imgs[1])))
print("Frame 1 + 4: {}".format(compare(imgs[0], imgs[3])))
print("Frame 3 + 4: {}".format(compare(imgs[2], imgs[3])))


#!/usr/bin/env python2.7

# load images
import glob
import os
import numpy as np
import cv2
from keras.models import model_from_json

with open("model.json", "r") as j:
    model = model_from_json(j.read())
model.load_weights("model.ker")



ibx = 0
listdir = list(os.listdir("/home/martin/Schreibtisch/ds_x/ds3"))
for im in sorted(listdir):
    if not im.endswith(".jpg"):
        continue

    print(im)
    ra = cv2.imread(os.path.join("/home/martin/Schreibtisch/ds_x/ds3", im))
    #print(img)
    #img = cv2.bilateralFilter(ra,14,100,100)
    img = cv2.GaussianBlur(ra, (9, 9), 0)
    #mask = cv2.inRange(img, (0, 120, 0), (160, 255, 160))
    #mask = cv2.erode(mask, None, iterations = 2)
    #mask = 255 - mask
    b, g, r = cv2.split(img)
    circles = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, 1, 100,
                               param1=50, param2=43, minRadius=15, maxRadius=200)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        try:
            for i in circles[0, :]:
                i[2] = i[2] + 3
                corp = ra[i[1] - i[2]-3:i[1] + i[2]+3, i[0] - i[2]-3:i[0] + i[2] +3]
                cv2.imshow("corp", corp)

                corp = cv2.resize(corp, (30, 30), interpolation=cv2.INTER_CUBIC)
                corp.reshape((1,) + corp.shape)
                print(corp.shape)

                p = model.predict(np.array([corp]), verbose=0)

                if p[0][0] >= 0.5 :
                    c = (0,255,0)

                    cv2.imwrite("/home/martin/Schreibtisch/ds_x/ds_pk2/d2nr%d6.jpg" % ibx, corp)
                else:
                    c = (0,0,255)

                    cv2.imwrite("/home/martin/Schreibtisch/ds_x/ds_nk2/d2nr%d6.jpg" % ibx, corp)
                print(p)
                # draw the outer circle
                cv2.circle(ra, (i[0], i[1]), i[2], c, 2)
                # draw the center of the circle
                cv2.circle(ra, (i[0], i[1]), 2, (0, 0, 255), 3)


                #cv2.waitKey(0)
                #cv2.imwrite("/home/martin/Schreibtisch/ds_x/ds_n/nr%d5.jpg" % ibx, corp)
                ibx += 1
        except cv2.error:
            continue

    cv2.imshow("img", ra)
    #cv2.imshow("mask", g)
    cv2.waitKey(1)



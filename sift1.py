#!/usr/bin/env python2
import numpy
import cv2
import silx
import time
from silx.image import sift

class DemoSift():
    def __init__(self):
        self.demoimage = [cv2.imread('frame1.png'),
                          cv2.imread('frame2.png'),
                          cv2.imread('frame3.png'),
                          cv2.imread('frame4.png'),]

        self.sift_ocl = sift.SiftPlan(template=self.demoimage[0], devicetype="CPU")
        #print("Device used for calculation: ", self.sift_ocl.ctx.devices[0].name)

    def compare(self, image1, image2):
        kp = self.sift_ocl(image1)
        kp1 = self.sift_ocl(image2)

        mp = sift.MatchPlan()
        match = mp(kp, kp1)

        # (Anzahl Keypoints Image1, Anzahl Keypoints Image2, Matched Keypoints)
        return (len(kp), len(kp1), match.shape[0])

    def test(self):
        print("Frame 1 + 2: {}".format(self.compare(self.demoimage[0], self.demoimage[1])))
        print("Frame 1 + 4: {}".format(self.compare(self.demoimage[0], self.demoimage[3])))
        print("Frame 3 + 4: {}".format(self.compare(self.demoimage[2], self.demoimage[3])))


    def show(self, image):
        cv2.imshow('image',image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def time(self, f):
        start = time.time()
        f()
        return (time.time() - start)

if __name__ == "__main__":
    demo = DemoSift()
    print(demo.time(demo.test))

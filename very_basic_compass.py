#!/usr/bin/env python2
import numpy
import cv2
import silx
import time
from silx.image import sift

class DemoSift():
    def __init__(self):
        self.cap = cv2.VideoCapture(0) 
        self.side_key_points = [None,None]

        initframe = self.cap.read()[1]

        self.sift_ocl = sift.SiftPlan(template=initframe, devicetype="CPU")
        self.mp = sift.MatchPlan()
        # print("Device used for calculation: ", self.sift_ocl.ctx.devices[0].name)

    def compare(self, image):
        kp = self.sift_ocl(image)

        matches = (self.mp(kp, self.side_key_points[0]).shape[0], 
                   self.mp(kp, self.side_key_points[1]).shape[0])

        # (Anzahl Keypoints Image1, Anzahl Keypoints Image2, Matched Keypoints)
        return matches


    def workloop(self):
        side = 0
        while True:
            ret, frame = self.cap.read()

            cv2.imshow("Video", frame)

            k = cv2.waitKey(1)

            if side < 2:
                # Wurde SPACE gedrueckt
                if k%256 == 32:
                    self.initside(side, frame)
                    side += 1
            else:
                # Beginnt Vergleich
                common_keys = self.compare(frame)
                # print(common_keys)

                if common_keys[0] > common_keys[1]:
                    visible_side = 0
                    print("SEITE 1")
                else:
                    visible_side = 1
                    print("SEITE 2")
                
            # Abbrechen mit ESC
            if k%256 == 27 or 0xFF == ord('q'):
                break
        self.cap.release() 
        cv2.destroyAllWindows()

    def initside(self, side, frame):
        print("Taken!!")
        self.side_key_points[side] = self.sift_ocl(frame)




if __name__ == "__main__":
    demo = DemoSift()
    demo.workloop()

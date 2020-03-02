#!/usr/bin/env python3
import time
import cv2
from threading import Thread

class Videocv():
    """
    Ensures constant frame rates for the CV2 video input. 
    """

    def __init__(self, src=0, fps=30):
        self.fps = float(fps)
        self._vc = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self._vc.read()
        self.ended = False
    
    def run(self):    
        Thread(target=self.get, args=()).start()
        return

    def get(self):
        while not self.ended:
            time.sleep(1/self.fps)
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self._vc.read()

    def stop(self):
        self.ended = True

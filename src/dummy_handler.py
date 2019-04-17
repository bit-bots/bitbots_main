#!/usr/bin/env python2
import cv2
import math
import yaml
import os
import time

from threading import Thread
from worker import BinaryCompassSift

class VisualCompassDummyHandler():
    """
    Docu
    """
    def __init__(self):
        dirname = os.path.dirname(__file__)
        relative_path = "../config/config.yaml"
        config_path = os.path.join(dirname, relative_path)

        with open(config_path, 'r') as stream:
            config = yaml.load(stream)

        source = config['dummy_handler']['input']

        if isinstance(source, basestring):
            root_folder = os.curdir
            source = root_folder + source
        
        self.video_getter = VideoGet(source).start()

        self.vc = BinaryCompassSift(config)

        self.loop()
    
    def debug_image_callback(self, debug_image):
        cv2.imshow("Video", debug_image)

    def data_callback(self, angle, confidence):
        print("Angle: {} | Confidence: {}".format(angle, confidence))
    
    def loop(self):
        side = 0
        while True:
            image = self.video_getter.frame

            k = cv2.waitKey(1)

            #TODO remove
            #self.debug_image_callback(image)

            if side < 2:
                self.debug_image_callback(image)
                # Wurde SPACE gedrueckt
                if k%256 == 32:
                    angle = side * math.pi
                    self.vc.set_truth(angle, image)
                    side += 1
            else:
                self.vc.process_image(image, resultCB=self.data_callback, debugCB=self.debug_image_callback)

            # Abbrechen mit ESC
            if k%256 == 27 or 0xFF == ord('q') or self.video_getter.stopped:
                break
        self.video_getter.stop()
        cv2.destroyAllWindows()


class VideoGet:
    """
    Class that continuously gets frames from a VideoCapture object
    with a dedicated thread. # TODO Umschreiben
    """

    def __init__(self, src=0):
        self.FPS = float(30)
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False
    
    def start(self):    
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            
            time.sleep(1/self.FPS)

            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()

    def stop(self):
        self.stopped = True

if __name__ == "__main__":
    VisualCompassDummyHandler()
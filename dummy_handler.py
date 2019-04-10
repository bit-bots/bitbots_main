#!/usr/bin/env python2
import cv2
import math
import yaml
import os
import time

from imutils.video import FileVideoStream
from imutils.video import FPS
from worker import VisualCompass

class VisualCompassDummyHandler():
    """
    Docu
    """
    def __init__(self):

        config_path = "config.yaml"

        with open(config_path, 'r') as stream:
            config = yaml.load(stream)

        source = config['dummy_handler']['input']

        if isinstance(source, basestring):
            root_folder = os.curdir
            source = root_folder + source
        
        self.fvs = FileVideoStream(source).start()

        self.vc = VisualCompass(config)

        self.loop()
    
    def debugImageCallback(self, debug_image):
        cv2.imshow("Video", debug_image)

    def dataCallback(self, angle, confidence):
        print("Angle: {} | Confidence: {}".format(angle, confidence))
    
    def loop(self):
        side = 0
        while self.fvs.more():
            image = self.fvs.read()

            k = cv2.waitKey(1)

            #TODO remove
            self.debugImageCallback(image)

            if side < 2:
                # Wurde SPACE gedrueckt
                if k%256 == 32:
                    angle = side * math.pi
                    self.vc.setTruth(angle, image)
                    side += 1
            else:
                self.vc.processImage(image, resultCB=self.dataCallback, debugCB=self.debugImageCallback)
                            
            # TODO echte time sync
            time.sleep(1/float(30))

            # Abbrechen mit ESC
            if k%256 == 27 or 0xFF == ord('q'):
                break
                
        cv2.destroyAllWindows()

if __name__ == "__main__":
    VisualCompassDummyHandler()
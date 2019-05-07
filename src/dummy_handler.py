#!/usr/bin/env python2

import cv2
import math
import yaml
import os
from videocv import Videocv
from worker import VisualCompass

try:
    string = basestring
except NameError:
    string = str

class VisualCompassDummyHandler():
    """
    Implements a Ros independent handler for a Visual Compass worker.
    """
    def __init__(self):
        dirname = os.path.dirname(__file__)
        relative_path = "../config/config.yaml"
        config_path = os.path.join(dirname, relative_path)

        with open(config_path, 'r') as stream:
            config = yaml.load(stream)

        source = config['dummy_handler_input']

        if isinstance(source, string):
            root_folder = os.curdir
            source = root_folder + source
        
        self.video_getter = Videocv(source)
        self.video_getter.run()

        self.vc = VisualCompass(config)

        self.loop(config)
    
    def debug_image_callback(self, debug_image):
        cv2.imshow("Video", debug_image)

    def data_callback(self, angle, confidence):
        print("Angle: {} | Confidence: {}".format(angle, confidence))
    
    def loop(self, config):
        side = 0
        while True:
            image = self.video_getter.frame

            k = cv2.waitKey(1)

            #TODO remove
            #self.debug_image_callback(image)

            sides = config['compass_multiple_sample_count'] if config['compass_type'] == 'multiple' else 2
            if side < sides:
                self.debug_image_callback(image)
                # Wurde SPACE gedrueckt
                if k%256 == 32:
                    angle = float(side) / sides * math.pi * 2
                    self.vc.set_truth(angle, image)
                    side += 1
            else:
                self.vc.process_image(image, resultCB=self.data_callback, debugCB=self.debug_image_callback)

            # Abbrechen mit ESC
            if k%256 == 27 or 0xFF == ord('q') or self.video_getter.ended:
                break
        self.video_getter.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    VisualCompassDummyHandler()
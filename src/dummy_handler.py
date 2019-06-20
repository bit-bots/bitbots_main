#!/usr/bin/env python2

import cv2
import math
import yaml
import os
import pickle
from os import path
from videocv import Videocv
from worker import VisualCompass
from key_point_converter import KeyPointConverter

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

        self.config = config

        source = config['dummy_handler_input']

        if isinstance(source, string):
            root_folder = os.curdir
            source = root_folder + source
        
        self.video_getter = Videocv(source)
        self.video_getter.run()

        self.vc = VisualCompass(config)

        self.loop(config)

    def save_ground_truth(self, ground_truth_file_path):
        # type (str) -> None
        """
        TODO docs
        """
        converter = KeyPointConverter()

        # get keypoints
        features = self.vc.get_ground_truth_features()

        # convert keypoints to basic values
        keypoints = features[0]
        keypoint_values = [converter.key_point2values(kp) for kp in keypoints]

        descriptors = features[1]

        meta = {
            'field': 1,
            'date': 0,
            'device': "flova",
            'compass_type': "multiple",
            'compass_matcher': "sift",
            'compass_multiple_ground_truth_images_count': 0,
            'keypoint_count': len(keypoint_values),
            'descriptor_count': len(descriptors)}

        dump_features = {
            'keypoint_values': keypoint_values, 
            'descriptors': descriptors,
            'meta': meta}

        # generate file path
        file_path = ground_truth_file_path
        # warn, if file does exist allready
        if path.isfile(file_path):
            print('Ground truth file at: %(path)s does ALLREADY EXIST. This will be overwritten.' % {'path': file_path})
        # save keypoints in pickle file
        with open(file_path, 'wb') as f:
            pickle.dump(dump_features, f)

    def load_ground_truth(self, ground_truth_file_path):
        # type: (str) -> ([], [])
        """
        TODO docs
        """
        # generate file path
        file_path = ground_truth_file_path
        features = ([], [])

        if path.isfile(file_path):
            # load keypoints of pickle file
            with open(file_path, 'rb') as f:
                features = pickle.load(f)
            print('Loaded ground truth file at: %(path)s' % {'path': file_path})

            keypoint_values = features['keypoint_values']
            descriptors = features['descriptors']
            meta = features['meta']

            # convert keypoint values to cv2 Keypoints
            keypoints = [cv2.KeyPoint(kp[0], kp[1], kp[2], kp[3], kp[4], kp[5], kp[6]) for kp in keypoint_values]

            return (keypoints, descriptors)
        else:
            print('NO ground truth file found at: %(path)s' % {'path': file_path})
    
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

            sides = config['compass_multiple_ground_truth_images_count'] if config['compass_type'] == 'multiple' else 2

            from_file = True
            if from_file:
                self.vc.set_ground_truth_features(self.load_ground_truth("test.txt"))

            if side < sides and not from_file:
                self.debug_image_callback(image)
                # Wurde SPACE gedrueckt
                if k%256 == 32:
                    angle = float(side) / sides * math.pi * 2
                    self.vc.set_truth(angle, image)
                    side += 1
                if side == (sides - 1):
                    self.save_ground_truth("test.txt")
            else:
                self.vc.process_image(image, resultCB=self.data_callback, debugCB=self.debug_image_callback)

            # Abbrechen mit ESC
            if k%256 == 27 or 0xFF == ord('q') or self.video_getter.ended:
                break
        self.video_getter.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    VisualCompassDummyHandler()

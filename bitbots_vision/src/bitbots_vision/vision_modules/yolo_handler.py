import cv2
import os
import rospy
import time
try:
    from pydarknet import Detector, Image
except:
    rospy.logerr("Not able to run YOLO!")
import numpy as np
from .candidate import CandidateFinder, Candidate

# todo implement candidate finder

class YoloHandler():
    def __init__(self, config, model_path):
        weightpath = os.path.join(model_path, "yolo_weights.weights")
        configpath = os.path.join(model_path, "config.cfg")
        datapath = os.path.join("/tmp/obj.data")
        namepath = os.path.join(model_path, "obj.names")

        self._generate_dummy_obj_data_file(namepath)

        self.config = config

        self.net = Detector(bytes(configpath, encoding="utf-8"), bytes(weightpath, encoding="utf-8"), 0.5,
                       bytes(datapath, encoding="utf-8"))
        self.classes = ["ball", "goalpost"]
        self.image = None
        self.results = None
        self.goalpost_candidates = None
        self.ball_candidates = None

    def _generate_dummy_obj_data_file(self, obj_name_path):
        obj_data = "classes = 2\nnames = " + obj_name_path
        with open('/tmp/obj.data', 'w') as f:
            f.write(obj_data)

    def set_image(self, img):
        if np.array_equal(img,self.image):
            return
        self.image = img
        self.results =  None
        self.image = img
        self.goalpost_candidates = None
        self.ball_candidates = None


    def predict(self):
        if self.results is None:
            self.results = self.net.detect(Image(self.image))
            class_ids = []
            confidences = []
            boxes = []
            self.ball_candidates = []
            self.goalpost_candidates = []
            for out in self.results:
                class_id = out[0]
                confidence = out[1]
                x, y, w, h = out[2]
                x = x - int(w // 2)
                y = y - int(h // 2)
                c = Candidate(int(x), int(y), int(w), int(h))
                c.rating = confidence
                class_id = class_id
                if class_id == b"ball":
                    self.ball_candidates.append(c)
                if class_id == b"goalpost":
                    self.goalpost_candidates.append(c)

    def get_candidates(self):
        self.predict()
        return [self.ball_candidates, self.goalpost_candidates]

class YoloBallDetector(CandidateFinder):

    def __init__(self, yolo):
        self.yolo = yolo

    def set_image(self, image):
        self.yolo.set_image(image)

    def get_candidates(self):
        return self.yolo.get_candidates()[0]

    def get_top_candidates(self, count = 1):
        ball_candidates = self.get_candidates()
        ball_candidates = Candidate.sort_candidates(ball_candidates)
        return ball_candidates[:count]

    def compute_top_candidate(self):
        pass

class YoloGoalpostDetector(CandidateFinder):

    def __init__(self, yolo):
        self.yolo = yolo

    def set_image(self, image):
        self.yolo.set_image(image)

    def get_candidates(self):
        return self.yolo.get_candidates()[1]

    def get_top_candidates(self, count = 1):
        ball_candidates = self.get_candidates()
        ball_candidates = Candidate.sort_candidates(ball_candidates)
        return ball_candidates[:count]

    def compute_top_candidate(self):
        pass


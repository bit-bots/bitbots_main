import cv2
import os
import rospy
try:
    from pydarknet import Detector, Image
except:
    rospy.logerr("Not able to run YOLO!")
import numpy as np
from candidate import CandidateFinder, Candidate

# todo implement candidate finder

class YoloHandler():
    def __init__(self, config, model_path):
        weightpath = os.path.join(model_path, "yolo_weights.weights")
        configpath = os.path.join(model_path, "config.cfg")
        datapath = os.path.join(model_path, "obj.data")

        self.config = config

        self.net = Detector(bytes(configpath, encoding="utf-8"), bytes(weightpath, encoding="utf-8"), 0,
                       bytes(datapath, encoding="utf-8"))
        self.classes = ["ball", "goalpost"]
        self.image = None
        self.COLORS = np.random.uniform(0, 255, size=(len(self.classes), 3))
        self.results = None
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4
        self.goalpost_candidates = None
        self.ball_candidates = None

    def set_image(self, img):
        if np.array_equal(img,self.image):
            return
        self.image = img
        self.results =  None
        self.image = Image(img)
        self.goalpost_candidates = None
        self.ball_candidates = None


    def predict(self):
        if self.results is None:
            results = self.net.detect(self.image)
            class_ids = []
            confidences = []
            boxes = []
            self.ball_candidates = []
            self.goalpost_candidates = []
            for out in results:
                print(out[2])
                class_id = out[0]
                confidence = out[1]
                x, y, w, h = out[2]
                boxes.append([x, y, w, h])
                confidences.append(confidence)
                class_ids.append(class_id)

            indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, self.nms_threshold)

            for i in indices:
                i = i[0]
                box = boxes[i]
                x = box[0]
                y = box[1]
                w = box[2]
                h = box[3]
                c = Candidate(x, y, w, h)
                c.rating = confidences[i]
                class_id = class_ids[i]
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


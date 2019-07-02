import cv2
import os
import argparse
import numpy as np
from .candidate import CandidateFinder, Candidate

# todo implement candidate finder

class YoloHandler():
    def __init__(self, config, model_path):
        weightpath = os.path.join(model_path, "yolo_weights.weights")
        configpath = os.path.join(model_path, "config.cfg")
        datapath = os.path.join(model_path, "obj.data")

        self.config = config
        self.classes = ["ball", "goalpost"]
        self.image = None
        self.width = None
        self.height = None
        self.blob = None
        self.COLORS = np.random.uniform(0, 255, size=(len(self.classes), 3))
        self.net = cv2.dnn.readNet(weightpath, configpath)
        self.outs = None
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4
        self.goalpost_candidates = None
        self.ball_candidates = None



    def get_output_layers(self, net):
        layer_names = net.getLayerNames()

        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

        return output_layers

    def set_image(self, img):
        if np.array_equal(img,self.image):
            return
        self.image = img
        self.width = img.shape[1]
        self.height = img.shape[0]
        self.blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0,0,0), True, crop=False)
        self.outs = None
        self.goalpost_candidates = None
        self.ball_candidates = None


    def predict(self):
        if self.outs is None:
            self.net.setInput(self.blob)
            self.outs = self.net.forward(self.get_output_layers(self.net))
            class_ids = []
            confidences = []
            boxes = []
            self.ball_candidates = []
            self.goalpost_candidates = []
            for out in self.outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:
                        center_x = int(detection[0] * self.width)
                        center_y = int(detection[1] * self.height)
                        w = int(detection[2] * self.width)
                        h = int(detection[3] * self.height)
                        x = center_x - w / 2
                        y = center_y - h / 2
                        class_ids.append(class_id)
                        confidences.append(float(confidence))
                        boxes.append([x, y, w, h])

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
                if class_id == 0:
                    self.ball_candidates.append(c)
                if class_id == 1:
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


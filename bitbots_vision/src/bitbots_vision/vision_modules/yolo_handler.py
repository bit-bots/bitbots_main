import cv2
import os
import abc
import rospy
import time
from profilehooks import profile
try:
    from pydarknet import Detector, Image
except ImportError:
    rospy.logerr("Not able to run Darknet YOLO! Its only executable under python3 with yolo34py or yolo34py-gpu installed.")
import numpy as np
from .candidate import CandidateFinder, BallDetector, Candidate

class YoloHandler():
    def __init__(self, config, model_path):
        raise NotImplementedError

    @abc.abstractmethod
    def set_image(self, img):
        """
        Image setter abstact method. (Cached)
        :param img: Image
        """
        raise NotImplementedError

    @abc.abstractmethod
    def predict(self):
        """
        Implemented version should run the neural metwork on the latest image. (Cached)
        """
        raise NotImplementedError

    def get_candidates(self):
        """
        Runs neural network and returns results for all classes. (Cached)
        """
        self.predict()
        return [self.ball_candidates, self.goalpost_candidates]

class YoloHandlerDarknet(YoloHandler):
    def __init__(self, config, model_path):
        weightpath = os.path.join(model_path, "yolo_weights.weights")
        configpath = os.path.join(model_path, "config.cfg")
        datapath = os.path.join("/tmp/obj.data")
        namepath = os.path.join(model_path, "obj.names")

        self._generate_dummy_obj_data_file(namepath)

        self.config = config

        self.net = Detector(bytes(configpath, encoding="utf-8"), bytes(weightpath, encoding="utf-8"), 0.5, bytes(datapath, encoding="utf-8"))
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
        if np.array_equal(img, self.image):
            return
        self.image = img
        self.results = None
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


class YoloHandlerOpenCV(YoloHandler):
    def __init__(self, config, model_path):
        weightpath = os.path.join(model_path, "yolo_weights.weights")
        configpath = os.path.join(model_path, "config.cfg")
        self.config = config
        self.classes = ["ball", "goalpost"]
        self.nms_threshold = 0.4
        self.confidence_threshold = 0.5
        self.image = None
        self.blob = None
        self.outs = None
        self.goalpost_candidates = None
        self.ball_candidates = None
        self.results = None
        self.COLORS = np.random.uniform(0, 255, size=(len(self.classes), 3))
        self.net = cv2.dnn.readNet(weightpath, configpath)

    @staticmethod
    def get_output_layers(net):
        layer_names = net.getLayerNames()

        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

        return output_layers

    def set_image(self, img):
        if np.array_equal(img, self.image):
            return
        self.image = img
        self.width = img.shape[1]
        self.height = img.shape[0]
        self.blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.outs = None
        self.goalpost_candidates = None
        self.ball_candidates = None

    def predict(self):
        if self.outs is None:
            self.net.setInput(self.blob)
            self.outs = self.net.forward(YoloHandlerOpenCV.get_output_layers(self.net))
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
                x = int(box[0])
                y = int(box[1])
                w = int(box[2])
                h = int(box[3])
                c = Candidate(x, y, w, h)
                c.rating = confidences[i]
                class_id = class_ids[i]
                if class_id == 0:
                    self.ball_candidates.append(c)
                if class_id == 1:
                    self.goalpost_candidates.append(c)


class YoloBallDetector(BallDetector):

    def __init__(self, yolo):
        self.yolo = yolo

    def set_image(self, image):
        self.yolo.set_image(image)

    def get_candidates(self):
        return self.yolo.get_candidates()[0]

    def get_top_candidates(self, count=1):
        ball_candidates = self.get_candidates()
        ball_candidates = Candidate.sort_candidates(ball_candidates)
        return ball_candidates[:count]

    def compute_top_candidate(self):
        self.yolo.predict()

class YoloGoalpostDetector(CandidateFinder):

    def __init__(self, yolo):
        self.yolo = yolo

    def set_image(self, image):
        self.yolo.set_image(image)

    def get_candidates(self):
        return self.yolo.get_candidates()[1]

    def get_top_candidates(self, count=1):
        ball_candidates = self.get_candidates()
        ball_candidates = Candidate.sort_candidates(ball_candidates)
        return ball_candidates[:count]

    def compute_top_candidate(self):
        self.yolo.predict()


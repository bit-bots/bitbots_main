import cv2
import argparse
import numpy as np
import Candidate

# todo implement candidate finder
class YoloHandler():
    def __init__(self, config, weight):
        configpath = "/home/jonas/git/bitbots/yolo/fork/darknet/cfg/yolov3-voc.cfg"
        weightpath = "/home/jonas/git/bitbots/yolo/fork/darknet/yolov3-voc_last.weights2"
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


    def draw_prediction(self, img, class_id, confidence, x, y, x_plus_w, y_plus_h):
        label = str(self.classes[class_id])


        color = self.COLORS[class_id]

        cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)

        cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)



    def set_image(self, img):
        self.image = img
        self.width = img.shape[1]
        self.height = img.shape[0]
        self.blob = cv2.dnn.blobFromImage(img, 1, (416, 416), (0,0,0), True, crop=False)
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
        return self.get_top_candidates()

    def get_top_candidates(self):
        self.ball_candidates = candidate.sort_candidates(self.ball_candidates)
        self.goalpost_candidates = candidate.sort_candidates(self.goalpost_candidates)
        return [self.ball_candidates[0], self.goalpost_candidates[0]]


    def get_best_ball_candidate(self):
        self.ball_candidates = candidate.sort_candidates(self.ball_candidates)
        return self.ball_candidates[0]

    def get_goalpost_candidates(self):
        self.goalpost_candidates = candidate.sort_candidates(self.goalpost_candidates)
        return self.goalpost_candidates

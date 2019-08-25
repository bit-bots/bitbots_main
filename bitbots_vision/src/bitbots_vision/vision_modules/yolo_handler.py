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
    """
    Defines a YoloHandler
    """
    def __init__(self, config, model_path):
        """
        Dummy constructor
        """
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
    """
    Yolo34py library implementation of our yolo model
    """
    def __init__(self, config, model_path):
        """
        Yolo constructor
        :param config: vision config dict
        :param model_path: path to the yolo model
        """
        # Define more paths
        weightpath = os.path.join(model_path, "yolo_weights.weights")
        configpath = os.path.join(model_path, "config.cfg")
        datapath = os.path.join("/tmp/obj.data")
        namepath = os.path.join(model_path, "obj.names")
        # Generates a dummy file for the library
        self._generate_dummy_obj_data_file(namepath)

        self.config = config

        # Setup detector
        self.net = Detector(bytes(configpath, encoding="utf-8"), bytes(weightpath, encoding="utf-8"), 0.5, bytes(datapath, encoding="utf-8"))
        # Set classes
        self.classes = ["ball", "goalpost"]
        # Set cached stuff
        self.image = None
        self.results = None
        self.goalpost_candidates = None
        self.ball_candidates = None

    def _generate_dummy_obj_data_file(self, obj_name_path):
        """
        Generates a dummy object data file.
        In which some meta information for the library is stored.
        :param obj_name_path: path to the class name file
        """
        # Generate file content
        obj_data = "classes = 2\nnames = " + obj_name_path
        # Write file
        with open('/tmp/obj.data', 'w') as f:
            f.write(obj_data)

    def set_image(self, image):
        """
        Set a image for yolo. This also resets the caches.
        :param image: current vision image
        """
        # Check if image has been processed
        if np.array_equal(image, self.image):
            return
        # Set image
        self.image = image
        # Reset cached stuff
        self.results = None
        self.goalpost_candidates = None
        self.ball_candidates = None

    def predict(self):
        """
        Runs the neural network
        """
        # Check if cached
        if self.results is None:
            # Run neural network
            self.results = self.net.detect(Image(self.image))
            # Init lists
            class_ids = []
            confidences = []
            boxes = []
            self.ball_candidates = []
            self.goalpost_candidates = []
            # Go through results
            for out in self.results:
                # Get class id
                class_id = out[0]
                # Get confidence
                confidence = out[1]
                # Get candidate position and size
                x, y, w, h = out[2]
                x = x - int(w // 2)
                y = y - int(h // 2)
                # Create candidate
                c = Candidate(int(x), int(y), int(w), int(h), confidence)
                # Append candidate to the right list depending on the class
                if class_id == b"ball":
                    self.ball_candidates.append(c)
                if class_id == b"goalpost":
                    self.goalpost_candidates.append(c)


class YoloHandlerOpenCV(YoloHandler):
    """
    Opencv library implementation of our yolo model
    """
    def __init__(self, config, model_path):
        # Build paths
        weightpath = os.path.join(model_path, "yolo_weights.weights")
        configpath = os.path.join(model_path, "config.cfg")
        # Set config
        self.config = config
        # Define classes
        self.classes = ["ball", "goalpost"]
        # Settings
        self.nms_threshold = 0.4
        self.confidence_threshold = 0.5
        # Setup neural network
        self.net = cv2.dnn.readNet(weightpath, configpath)
        # Set default state to all cached values
        self.image = None
        self.blob = None
        self.outs = None
        self.goalpost_candidates = None
        self.ball_candidates = None
        self.results = None

    @staticmethod
    def get_output_layers(net):
        """
        Library stuff
        """
        layer_names = net.getLayerNames()

        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

        return output_layers

    def set_image(self, image):
        """
        Set a image for yolo. This also resets the caches.
        :param image: current vision image
        """
        # Check if image has been processed
        if np.array_equal(image, self.image):
            return
        # Set image
        self.image = image
        self.image = image
        self.width = image.shape[1]
        self.height = image.shape[0]
        # Create blob
        self.blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        # Reset cached stuff
        self.outs = None
        self.goalpost_candidates = None
        self.ball_candidates = None

    def predict(self):
        """
        Runs the neural network
        """
        # Check if cached
        if self.outs is None:
            # Set image
            self.net.setInput(self.blob)
            # Run net
            self.outs = self.net.forward(YoloHandlerOpenCV.get_output_layers(self.net))
            # Create lists
            class_ids = []
            confidences = []
            boxes = []
            self.ball_candidates = []
            self.goalpost_candidates = []
            # Iterate over output/detections
            for out in self.outs:
                for detection in out:
                    # Get score
                    scores = detection[5:]
                    # Ger class
                    class_id = np.argmax(scores)
                    # Get confidence from score
                    confidence = scores[class_id]
                    # Static threshold
                    if confidence > 0.5:
                        # Get center point of the candidate
                        center_x = int(detection[0] * self.width)
                        center_y = int(detection[1] * self.height)
                        # Get the heigh/width
                        w = int(detection[2] * self.width)
                        h = int(detection[3] * self.height)
                        # Calc the upper left point
                        x = center_x - w / 2
                        y = center_y - h / 2
                        # Append result
                        class_ids.append(class_id)
                        confidences.append(float(confidence))
                        boxes.append([x, y, w, h])

            # Merge boxes
            indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, self.nms_threshold)

            # Iterate over filtered boxes
            for i in indices:
                # Get id
                i = i[0]
                # Get box
                box = boxes[i]
                # Convert the box position/size to int
                x = int(box[0])
                y = int(box[1])
                w = int(box[2])
                h = int(box[3])
                # Create the candidate
                c = Candidate(x, y, w, h, confidences[i])
                # Append candidate to the right list depending on the class
                class_id = class_ids[i]
                if class_id == 0:
                    self.ball_candidates.append(c)
                if class_id == 1:
                    self.goalpost_candidates.append(c)


class YoloBallDetector(BallDetector):
    """
    A ball detector using the yolo neural network
    """
    def __init__(self, config, yolo):
        """
        :param config: The vision config
        :param yolo: An YoloHandler implementation that runs the yolo network
        """
        # Set the yolo network
        self.yolo = yolo
        # Set the config. Not needed at the moment
        self.config = config

    def set_image(self, image):
        """
        Set a image for yolo. This is cached.
        :param image: current vision image
        """
        self.yolo.set_image(image)

    def get_candidates(self):
        """
        :return: all found ball candidates
        """
        return self.yolo.get_candidates()[0]

    def compute(self):
        """
        Runs the yolo network
        """
        self.yolo.predict()

class YoloGoalpostDetector(CandidateFinder):
    """
    A goalpost detector using the yolo neural network
    """
    def __init__(self, yolo):
        """
        :param yolo: An YoloHandler implementation that runs the yolo network
        """
        self.yolo = yolo

    def set_image(self, image):
        """
        Set a image for yolo. This is cached.
        :param image: current vision image
        """
        self.yolo.set_image(image)

    def get_candidates(self):
        """
        :return: all found goalpost candidates
        """
        return self.yolo.get_candidates()[1]

    def compute(self):
        """
        Runs the yolo network
        """
        self.yolo.predict()


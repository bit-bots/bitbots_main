import cv2
import os
import abc
import rospy
import numpy as np
from math import exp
try:
    from pydarknet import Detector, Image
except ImportError:
    rospy.logerr("Not able to run Darknet YOLO! Its only executable under python3 with yolo34py or yolo34py-gpu installed.", logger_name="vision_yolo")
try:
    from openvino.inference_engine import IENetwork, IECore
except ImportError:
    rospy.logerr("Not able to run YOLO on the Intel NCS2 TPU! The OpenVINO SDK should be installed to run this hardware acceleration", logger_name="vision_yolo")
from .candidate import CandidateFinder, BallDetector, Candidate


class YoloHandler():
    """
    Defines an abstract YoloHandler
    """
    def __init__(self, config, model_path):
        """
        Init abstract YoloHandler.
        """
        self._ball_candidates = None
        self._goalpost_candidates = None

        # Set if values should be cached
        self._caching = config['caching']

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
        return [self.get_ball_candidates(), self.get_goalpost_candidates()]

    def get_ball_candidates(self):
        """
        Runs neural network and returns results for ball class. (Cached)
        """
        self.predict()
        return self._ball_candidates

    def get_goalpost_candidates(self):
        """
        Runs neural network and returns results for goalpost class. (Cached)
        """
        self.predict()
        return self._goalpost_candidates

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

        self._config = config

        # Setup detector
        self._net = Detector(bytes(configpath, encoding="utf-8"), bytes(weightpath, encoding="utf-8"), 0.5, bytes(datapath, encoding="utf-8"))
        # Set cached stuff
        self._image = None
        self._results = None
        super(YoloHandlerDarknet, self).__init__(config, model_path)

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
        if np.array_equal(image, self._image):
            return
        # Set image
        self._image = image
        # Reset cached stuff
        self._results = None
        self._goalpost_candidates = None
        self._ball_candidates = None

    def predict(self):
        """
        Runs the neural network
        """
        # Check if cached
        if self._results is None or not self._caching:
            # Run neural network
            self._results = self._net.detect(Image(self._image))
            # Init lists
            self._ball_candidates = []
            self._goalpost_candidates = []
            # Go through results
            for out in self._results:
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
                    self._ball_candidates.append(c)
                if class_id == b"goalpost":
                    self._goalpost_candidates.append(c)

class YoloHandlerOpenCV(YoloHandler):
    """
    Opencv library implementation of our yolo model
    """
    def __init__(self, config, model_path):
        # Build paths
        weightpath = os.path.join(model_path, "yolo_weights.weights")
        configpath = os.path.join(model_path, "config.cfg")
        # Set config
        self._config = config
        # Settings
        self._nms_threshold = 0.4
        self._confidence_threshold = 0.5
        # Setup neural network
        self._net = cv2.dnn.readNet(weightpath, configpath)
        # Set default state to all cached values
        self._image = None
        self._blob = None
        self._outs = None
        self._results = None
        super(YoloHandlerOpenCV, self).__init__(config, model_path)

    def _get_output_layers(self):
        """
        Library stuff
        """
        layer_names = self._net.getLayerNames()

        output_layers = [layer_names[i[0] - 1] for i in self._net.getUnconnectedOutLayers()]

        return output_layers

    def set_image(self, image):
        """
        Set a image for yolo. This also resets the caches.

        :param image: current vision image
        """
        # Check if image has been processed
        if np.array_equal(image, self._image):
            return
        # Set image
        self._image = image
        self._width = image.shape[1]
        self._height = image.shape[0]
        # Create blob
        self._blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        # Reset cached stuff
        self._outs = None
        self._goalpost_candidates = None
        self._ball_candidates = None

    def predict(self):
        """
        Runs the neural network
        """
        # Check if cached
        if self._outs is None or not self._caching:
            # Set image
            self._net.setInput(self._blob)
            # Run net
            self._outs = self._net.forward(self._get_output_layers())
            # Create lists
            class_ids = []
            confidences = []
            boxes = []
            self._ball_candidates = []
            self._goalpost_candidates = []
            # Iterate over output/detections
            for out in self._outs:
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
                        center_x = int(detection[0] * self._width)
                        center_y = int(detection[1] * self._height)
                        # Get the heigh/width
                        w = int(detection[2] * self._width)
                        h = int(detection[3] * self._height)
                        # Calc the upper left point
                        x = center_x - w / 2
                        y = center_y - h / 2
                        # Append result
                        class_ids.append(class_id)
                        confidences.append(float(confidence))
                        boxes.append([x, y, w, h])

            # Merge boxes
            indices = cv2.dnn.NMSBoxes(boxes, confidences, self._confidence_threshold, self._nms_threshold)

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
                    self._ball_candidates.append(c)
                if class_id == 1:
                    self._goalpost_candidates.append(c)

class YoloHandlerNCS2(YoloHandler):
    """
    This Code is based on a code example from the Intel documentation under following licensing:

    Copyright (C) 2018-2019 Intel Corporation

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
    """

    def __init__(self, config, model_path):
        # Create model file paths
        model_xml = os.path.join(model_path, "yolo.xml")
        model_bin = os.path.join(model_path, "yolo.bin")

        # Plugin initialization
        rospy.loginfo("Creating Inference Engine...")
        ie = IECore()

        # Reading the IR generated by the Model Optimizer (.xml and .bin files)
        rospy.loginfo("Loading network files:\n\t{}\n\t{}".format(model_xml, model_bin))
        self._net = IENetwork(model=model_xml, weights=model_bin)

        assert len(self._net.inputs.keys()) == 1, "Sample supports only YOLO V3 based single input topologies"

        # Preparing network inputs
        rospy.loginfo("Preparing inputs")
        self._input_blob = next(iter(self._net.inputs))

        #  Defaulf batch_size is 1
        self._net.batch_size = 1

        # Read and pre-process input images
        self._n, self._c, self._h, self._w = self._net.inputs[self._input_blob].shape

        # Device type
        device = "MYRIAD"

        # Loading model to the plugin
        rospy.loginfo("Loading model to the plugin")
        self._exec_net = ie.load_network(network=self._net, num_requests=2, device_name=device)

        # Params TODO
        self._prob_threshold = 0.5

        self._iou_threshold = 0.4

        self._image = None
        self._caching = True

    def set_image(self, image):
        """
        Set a image for yolo. This also resets the caches.

        :param image: current vision image
        """
        # Check if image has been processed
        if np.array_equal(image, self._image):
            return
        # Set image
        self._image = image
        self._goalpost_candidates = None
        self._ball_candidates = None

    def _entry_index(self, side, coord, classes, location, entry):
        """
        Calculates the index of a yolo object.
        """
        side_power_2 = side ** 2
        n = location // side_power_2
        loc = location % side_power_2
        return int(side_power_2 * (n * (coord + classes + 1) + entry) + loc)

    def _scale_bbox(self, x, y, h, w, class_id, confidence, h_scale, w_scale):
        """
        Scales bounding box to original image size and
        changes its representation from x, y, h, w to xmin, xmax, ymin, ymax.

        :param x: x position in yolo image shape
        :param y: y position in yolo image shape
        :param h: height in yolo image shape
        :param w: width in yolo image shape
        :param class_id: Id of the class
        :param confidence: Confidence of the bounding box
        :param h_scale: Scale from the yolo image shape to the original image shape in vertical direction
        :param w_scale: Scale from the yolo image shape to the original image shape in horizontal direction
        :return: Bounding box as dict with keys 'xmin', 'xmax', 'ymin', 'ymax', 'class_id', 'confidence'.
        """

        xmin = int((x - w / 2) * w_scale)
        ymin = int((y - h / 2) * h_scale)
        xmax = int(xmin + w * w_scale)
        ymax = int(ymin + h * h_scale)
        return dict(xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax, class_id=class_id, confidence=confidence)

    def _parse_yolo_region(self, blob, resized_image_shape, original_im_shape, params, threshold):
        """
        Parses bounding boxes out of an yolo output layer.

        :param blob: Yolo layer output blob
        :param resized_image_shape: Yolo input image shape
        :param original_im_shape: Vision image shape
        :params: Layer parameters
        :param threshold: Yolo bounding box threshold
        :return: List of bounding boxes
        """
        # Validating output parameters
        _, _, out_blob_h, out_blob_w = blob.shape
        assert out_blob_w == out_blob_h, "Invalid size of output blob. It sould be in NCHW layout and height should " \
                                        "be equal to width. Current height = {}, current width = {}" \
                                        "".format(out_blob_h, out_blob_w)

        # Extracting layer parameters
        orig_im_h, orig_im_w = original_im_shape
        resized_image_h, resized_image_w = resized_image_shape
        objects = list()
        predictions = blob.flatten()
        side_square = params.side * params.side

        # Parsing YOLO Region output
        for i in range(side_square):
            row = i // params.side
            col = i % params.side
            for n in range(params.num):
                obj_index = self._entry_index(params.side, params.coords, params.classes, n * side_square + i, params.coords)
                scale = predictions[obj_index]
                if scale < threshold:
                    continue
                box_index = self._entry_index(params.side, params.coords, params.classes, n * side_square + i, 0)
                # Network produces location predictions in absolute coordinates of feature maps.
                # Scale it to relative coordinates.
                x = (col + predictions[box_index + 0 * side_square]) / params.side
                y = (row + predictions[box_index + 1 * side_square]) / params.side
                # Value for exp is very big number in some cases so following construction is using here
                try:
                    w_exp = exp(predictions[box_index + 2 * side_square])
                    h_exp = exp(predictions[box_index + 3 * side_square])
                except OverflowError:
                    continue
                # Depending on topology we need to normalize sizes by feature maps (up to YOLOv3) or by input shape (YOLOv3)
                w = w_exp * params.anchors[2 * n] / (resized_image_w if params.isYoloV3 else params.side)
                h = h_exp * params.anchors[2 * n + 1] / (resized_image_h if params.isYoloV3 else params.side)
                for j in range(params.classes):
                    class_index = self._entry_index(params.side, params.coords, params.classes, n * side_square + i,
                                            params.coords + 1 + j)
                    confidence = scale * predictions[class_index]
                    if confidence < threshold:
                        continue
                    objects.append(self._scale_bbox(x=x, y=y, h=h, w=w, class_id=j, confidence=confidence,
                                            h_scale=orig_im_h, w_scale=orig_im_w))
        return objects

    def _intersection_over_union(self, box_1, box_2):
        """
        Calculates an iou of two bounding boxes.

        :param box_1: first bounding box
        :param box_2: second bounding box
        :return: iou value
        """
        # Calculate overlap
        width_of_overlap_area = min(box_1['xmax'], box_2['xmax']) - max(box_1['xmin'], box_2['xmin'])
        height_of_overlap_area = min(box_1['ymax'], box_2['ymax']) - max(box_1['ymin'], box_2['ymin'])
        if width_of_overlap_area < 0 or height_of_overlap_area < 0:
            area_of_overlap = 0
        else:
            area_of_overlap = width_of_overlap_area * height_of_overlap_area
        # Calculate areas
        box_1_area = (box_1['ymax'] - box_1['ymin']) * (box_1['xmax'] - box_1['xmin'])
        box_2_area = (box_2['ymax'] - box_2['ymin']) * (box_2['xmax'] - box_2['xmin'])
        # Calculate union
        area_of_union = box_1_area + box_2_area - area_of_overlap
        # Catch divide by zero
        if area_of_union == 0:
            return 0
        # Return iou
        return area_of_overlap / area_of_union

    def _scaled_bbox_to_candidate(self, bbox):
        return Candidate(
            int(bbox['xmin']),
            int(bbox['ymin']),
            int(bbox['xmax']) - int(bbox['xmin']),
            int(bbox['ymax']) - int(bbox['ymin']),
            bbox['confidence']
            )

    def predict(self):
        if (self._ball_candidates is None and self._goalpost_candidates is None) or not self._caching:
            # Set up variables
            self._ball_candidates = list()
            self._goalpost_candidates = list()

            rospy.logdebug("Starting inference...")

            # Set request id for the stick. Since we only make one call at a time, we use a static parameter.
            request_id = 1
            # Resize image to yolo input size
            in_frame = cv2.resize(self._image, (self._w, self._h))

            # resize input_frame to network size
            in_frame = in_frame.transpose((2, 0, 1))  # Change data layout from HWC to CHW
            in_frame = in_frame.reshape((self._n, self._c, self._h, self._w))

            # Start inference
            self._exec_net.start_async(request_id=request_id, inputs={self._input_blob: in_frame})

            # Collecting object detection results
            objects = list()
            # Create barrier. This lets all following processing steps wait until the prediction is calculated.
            if self._exec_net.requests[request_id].wait(-1) == 0:
                # Get output
                output = self._exec_net.requests[request_id].outputs
                # Iterate over output layers
                for layer_name, out_blob in output.items():
                    # Reshape output layer
                    out_blob = out_blob.reshape(self._net.layers[self._net.layers[layer_name].parents[0]].shape)
                    # Create layer params object
                    layer_params = YoloParams(self._net.layers[layer_name].params, out_blob.shape[2])
                    # Parse yolo bounding boxes out of output blob
                    objects.extend(
                        self._parse_yolo_region(
                            out_blob,
                            in_frame.shape[2:],
                            self._image.shape[:-1],
                            layer_params,
                            self._prob_threshold))

            # Filtering overlapping boxes
            objects = sorted(objects, key=lambda obj : obj['confidence'], reverse=True)
            for i in range(len(objects)):
                if objects[i]['confidence'] == 0:
                    continue
                for j in range(i + 1, len(objects)):
                    if self._intersection_over_union(objects[i], objects[j]) > self._iou_threshold:
                        objects[j]['confidence'] = 0

            # Convert objects to candidates
            for yolo_object in objects:
                # Drop out mostly 0 confidence objects
                if yolo_object['confidence'] > self._prob_threshold:
                    # Sort detections in classes
                    if yolo_object['class_id'] == 0:
                        self._ball_candidates.append(
                            self._scaled_bbox_to_candidate(yolo_object)
                        )
                    if yolo_object['class_id'] == 1:
                        self._goalpost_candidates.append(
                            self._scaled_bbox_to_candidate(yolo_object)
                        )


class YoloParams:
        """
        Class to store params of yolo layers
        """
        def __init__(self, param, side):
            self.num = 3 if 'num' not in param else int(param['num'])
            self.coords = 4 if 'coords' not in param else int(param['coords'])
            self.classes = 2 if 'classes' not in param else int(param['classes'])
            self.anchors = [10.0, 13.0, 16.0, 30.0, 33.0, 23.0, 30.0, 61.0, 62.0, 45.0, 59.0, 119.0, 116.0, 90.0, 156.0,
                            198.0,
                            373.0, 326.0] if 'anchors' not in param else [float(a) for a in param['anchors'].split(',')]

            if 'mask' in param:
                mask = [int(idx) for idx in param['mask'].split(',')]
                self.num = len(mask)

                maskedAnchors = []
                for idx in mask:
                    maskedAnchors += [self.anchors[idx * 2], self.anchors[idx * 2 + 1]]
                self.anchors = maskedAnchors

            self.side = side
            self.isYoloV3 = 'mask' in param  # Weak way to determine but the only one.


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
        self._yolo = yolo
        # Set the config. Not needed at the moment
        self._config = config

    def set_image(self, image):
        """
        Set a image for yolo. This is cached.

        :param image: current vision image
        """
        self._yolo.set_image(image)

    def get_candidates(self):
        """
        :return: all found ball candidates
        """
        return self._yolo.get_ball_candidates()

    def compute(self):
        """
        Runs the yolo network
        """
        self._yolo.predict()

class YoloGoalpostDetector(CandidateFinder):
    """
    A goalpost detector using the yolo neural network
    """
    def __init__(self, config, yolo):
        """
        :param config: The vision config
        :param yolo: An YoloHandler implementation that runs the yolo network
        """
        self._config = config
        self._yolo = yolo

    def set_image(self, image):
        """
        Set a image for yolo. This is cached.

        :param image: current vision image
        """
        self._yolo.set_image(image)

    def get_candidates(self):
        """
        :return: all found goalpost candidates
        """
        return self._yolo.get_goalpost_candidates()

    def compute(self):
        """
        Runs the yolo network
        """
        self._yolo.predict()

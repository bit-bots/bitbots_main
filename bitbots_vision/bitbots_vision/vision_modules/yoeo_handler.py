import torch
import yoeo.utils.utils

from .candidate import CandidateFinder, Candidate
from .field_boundary import IFieldDetector

from abc import ABC, abstractmethod
from collections import defaultdict
import cv2
import numpy as np
import os
from rclpy import logging
from typing import List, Union, Dict, Any
import yaml

logger = logging.get_logger('vision_yoeo')

try:
    from yoeo import models as torch_models, detect as torch_detect
except ImportError:
    logger.error("Unable to import pytorchyolo. This might be fine if you use another neural network type.")

try:
    from openvino.runtime import Core as openvino_runtime_Core
except ImportError:
    logger.error("Unable to import openvino. This might be fine if you use another neural network type.")


class IYOEOHandler(ABC):
    @abstractmethod
    def set_config(self, config: Dict) -> None:
        ...

    @abstractmethod
    def get_detection_candidate_class_names(self) -> List[str]:
        ...

    @abstractmethod
    def get_detection_candidates_for(self, class_name: str) -> List[Candidate]:
        ...

    @abstractmethod
    def get_segmentation_candidate_class_names(self) -> List[str]:
        ...

    @abstractmethod
    def get_segmentation_mask_for(self, class_name: str):
        """
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        ...

    @abstractmethod
    def predict(self) -> None:
        ...

    @abstractmethod
    def set_image(self, img) -> None:
        ...


class YOEOHandlerTemplate(IYOEOHandler):
    def __init__(self, config: Dict, model_path: str):
        self._use_caching: Union[None, bool] = None

        self._image = None  # : Union[None, numpy.ndarray]
        self._iou_non_max_suppression_thresh: Union[None, float] = None

        self._det_candidates: Dict = defaultdict(list)
        self._det_class_names: Union[None, List[str]] = None
        self._det_confidence_thresh: Union[None, float] = None

        self.set_config(config)

        self._seg_masks: Dict = dict()
        self._seg_class_names: Union[None, List[str]] = None

        self._load_candidate_class_names(model_path)

    def set_config(self, config: Dict) -> None:
        self._use_caching = config['caching']
        self._iou_non_max_suppression_thresh = config['yoeo_nms_threshold']
        self._det_confidence_thresh = config['yoeo_conf_threshold']

    def _load_candidate_class_names(self, model_path: str) -> None:
        path = os.path.join(model_path, "yoeo_names.yaml")

        with open(path, 'r', encoding="utf-8") as fp:
            class_names = yaml.load(fp, Loader=yaml.SafeLoader)

        assert "detection" in class_names.keys(), f"Missing key 'detection' in {path}"
        assert "segmentation" in class_names.keys(), f"Missing key 'segmentation' in {path}"

        self._det_class_names = class_names['detection']
        self._seg_class_names = class_names['segmentation']

    def _add_detection_candidates_for(self, class_name: str, candidates: List[Candidate]) -> None:
        self._det_candidates[class_name] = list(candidates)

    def _add_segmentation_mask_for(self, class_name: str, mask) -> None:
        self._seg_masks[class_name] = mask

    def get_detection_candidates_for(self, class_name: str) -> List[Candidate]:
        assert class_name in self._det_class_names, \
            f"Class '{class_name}' is not available for the current YOEO model (detection)"

        self.predict()

        return self._det_candidates[class_name]

    def predict(self) -> None:
        if self._prediction_has_to_be_updated():
            logger.debug(f"Computing new prediction...")
            self._compute_new_prediction(
                self._image,
                self._det_confidence_thresh,
                self._iou_non_max_suppression_thresh
            )

    def _prediction_has_to_be_updated(self) -> bool:
        return not self._use_caching or not (self._seg_masks or self._det_candidates)

    def get_detection_candidate_class_names(self) -> List[str]:
        return self._det_class_names

    def get_segmentation_candidate_class_names(self) -> List[str]:
        return self._seg_class_names

    def get_segmentation_mask_for(self, class_name: str):
        """
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        assert class_name in self._seg_class_names, \
            f"Class '{class_name}' ist not available for the current YOEO model (segmentation)"

        self.predict()

        return self._seg_masks[class_name]

    def set_image(self, img) -> None:
        self._reset()
        self._update_image(img)

    def _reset(self) -> None:
        self._det_candidates = defaultdict(list)
        self._seg_masks = dict()

    def _update_image(self, img) -> None:
        self._image = img

    @abstractmethod
    def _compute_new_prediction(self, image, conf_thresh, nms_thres) -> None:
        """
        Method must fill
            - self._det_candidates (use self._add_detection_candidates_for(...))
            - self._seg_masks (use self._add_segmentation_mask_for(...))
        """
        ...


class YOEOHandlerOpenVino(YOEOHandlerTemplate):
    # class _YoloParams:
    #     """
    #     Class to store params of yolo layers
    #     """
    #
    #     def __init__(self, param, side):
    #         self.num = 3 if 'num' not in param else int(param['num'])
    #         self.coords = 4 if 'coords' not in param else int(param['coords'])
    #         self.classes = 2 if 'classes' not in param else int(param['classes'])
    #         self.anchors = [10.0, 13.0, 16.0, 30.0, 33.0, 23.0, 30.0, 61.0, 62.0, 45.0, 59.0, 119.0, 116.0, 90.0, 156.0,
    #                         198.0,
    #                         373.0, 326.0] if 'anchors' not in param else [float(a) for a in param['anchors'].split(',')]
    #
    #         if 'mask' in param:
    #             mask = [int(idx) for idx in param['mask'].split(',')]
    #             self.num = len(mask)
    #
    #             maskedAnchors = []
    #             for idx in mask:
    #                 maskedAnchors += [self.anchors[idx * 2], self.anchors[idx * 2 + 1]]
    #             self.anchors = maskedAnchors
    #
    #         self.side = side
    #         self.isYoloV3 = 'mask' in param  # Weak way to determine but the only one.

    def __init__(self, config: Dict, model_path: str):
        logger.debug(f"Entry {self.__class__.__name__} Constructor")

        super().__init__(config, model_path)
        model_xml = os.path.join(model_path, "yoeo.xml")
        model_bin = os.path.join(model_path, "yoeo.bin")

        # https://docs.openvino.ai/latest/notebooks/002-openvino-api-with-output.html (April 9, 2022)
        self._inference_engine = openvino_runtime_Core()

        logger.debug(f"Loading network...\n\t{model_xml}\n\t{model_bin}")
        #model = self._inference_engine.read_model(model=model_xml, weights=model_bin)
        device = self._select_device()

        logger.debug(f"Compiling network on device '{device}'...")
        #self._compiled_model = self._inference_engine.compile_model(model=model, device_name=device)

        model_onnx = os.path.join(model_path, "yoeo.onnx")
        model = self._inference_engine.read_model(model=model_onnx)
        self._compiled_model = self._inference_engine.compile_model(model=model, device_name=device)

        self._input_layer = self._compiled_model.inputs[0]
        self._output_layer_detections = self._compiled_model.outputs[0]
        self._output_layer_segmentations = self._compiled_model.outputs[1]

        # Read input layer shape for image preprocessing
        self._n, self._c, self._h, self._w = self._input_layer.shape

        logger.debug(f"Exit {self.__class__.__name__} Constructor")

    def _select_device(self) -> str:
        if "MYRIAD" in self._inference_engine.available_devices:  # NCS2 stick
            device = "MYRIAD"
        else:
            device = "CPU"
        return device

        # #  Defaulf batch_size is 1
        # self._net.batch_size = 1
        #

    # def _entry_index(self, side, coord, classes, location, entry):
    #     """
    #     Calculates the index of a yolo object.
    #     """
    #     side_power_2 = side ** 2
    #     n = location // side_power_2
    #     loc = location % side_power_2
    #     return int(side_power_2 * (n * (coord + classes + 1) + entry) + loc)

    # def _parse_yolo_region(self, blob, resized_image_shape, original_im_shape, params, threshold):
    #     """
    #     Parses bounding boxes out of an yolo output layer.
    #
    #     :param blob: Yolo layer output blob
    #     :param resized_image_shape: Yolo input image shape
    #     :param original_im_shape: Vision image shape
    #     :param params: Layer parameters
    #     :param threshold: Yolo bounding box threshold
    #     :return: List of bounding boxes
    #     """
    #     # Validating output parameters
    #     _, _, out_blob_h, out_blob_w = blob.shape
    #     assert out_blob_w == out_blob_h, \
    #         f"Invalid size of output blob. It should be in NCHW layout and height should be equal to width. Current height: '{out_blob_h}', current width = '{out_blob_w}'"
    #
    #     # Extracting layer parameters
    #     original_image_height, original_image_width = original_im_shape
    #     resized_image_h, resized_image_w = resized_image_shape
    #     objects = list()
    #     predictions = blob.flatten()
    #     side_square = params.side ** 2
    #
    #     # Parsing YOLO Region output
    #     for i in range(side_square):
    #         row = i // params.side
    #         col = i % params.side
    #         for n in range(params.num):
    #             obj_index = self._entry_index(params.side, params.coords, params.classes, n * side_square + i,
    #                                           params.coords)
    #             scale = predictions[obj_index]
    #             # Skip unrealistic boxes
    #             if scale < threshold:
    #                 continue
    #             box_index = self._entry_index(params.side, params.coords, params.classes, n * side_square + i, 0)
    #             # Network produces location predictions in absolute coordinates of feature maps.
    #             # Scale it to relative coordinates.
    #             x = (col + predictions[box_index + 0 * side_square]) / params.side
    #             y = (row + predictions[box_index + 1 * side_square]) / params.side
    #             # Value for exp might be a very large number, so the following construction is used here
    #             try:
    #                 w_exp = exp(predictions[box_index + 2 * side_square])
    #                 h_exp = exp(predictions[box_index + 3 * side_square])
    #             except OverflowError:
    #                 continue
    #             # Depending on topology we need to normalize sizes by feature maps (up to YOLOv3) or by input shape (YOLOv3)
    #             w = w_exp * params.anchors[2 * n] / (resized_image_w if params.isYoloV3 else params.side)
    #             h = h_exp * params.anchors[2 * n + 1] / (resized_image_h if params.isYoloV3 else params.side)
    #             # Iterate over classes
    #             for j in range(params.classes):
    #                 class_index = self._entry_index(params.side, params.coords, params.classes, n * side_square + i,
    #                                                 params.coords + 1 + j)
    #                 confidence = scale * predictions[class_index]
    #                 # Skip box if confidence in class is too low
    #                 if confidence < threshold:
    #                     continue
    #                 h = int(h * original_image_height)
    #                 w = int(w * original_image_width)
    #                 x = x * original_image_width - w / 2
    #                 y = y * original_image_height - h / 2
    #                 list_of_coordinates = [int(x), int(y), int(w), int(h)]
    #                 # Convert to int
    #                 objects.append([list_of_coordinates, float(confidence), j])
    #     return objects

    def _compute_new_prediction(self, image, conf_thresh, nms_thres) -> None:
        # Set request id for the stick. Since we only make one call at a time, we use a static parameter.
        request_id = 1
        # Resize image to yoeo input size

        in_frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        in_frame = in_frame.astype(np.float64)
        in_frame = in_frame / 255
        v_padding = (np.max(in_frame.shape) - in_frame.shape[0]) // 2
        h_padding = (np.max(in_frame.shape) - in_frame.shape[1]) // 2
        in_frame = cv2.copyMakeBorder(in_frame, v_padding, v_padding, h_padding, h_padding, cv2.BORDER_CONSTANT, 0)
        in_frame = cv2.resize(in_frame, (self._w, self._h))

        # resize input_frame to network size
        in_frame = in_frame.transpose((2, 0, 1))  # Change data layout from HWC to CHW
        in_frame = in_frame.reshape((self._n, self._c, self._h, self._w))

        network_output = self._compiled_model(inputs=[in_frame])
        detections = network_output[self._output_layer_detections]
        segmentations = network_output[self._output_layer_segmentations]

        self._process_segmentation_output(segmentations, image.shape[:2])
        self._process_detections(detections, .5, .5, image.shape[:2])

    def _process_segmentation_output(self, segmentations: Any, original_image_dims) -> None:
        segmentations = torch.from_numpy(segmentations)
        segmentations = yoeo.utils.utils.rescale_segmentations(segmentations, 416, original_image_dims)
        segmentations = np.moveaxis(segmentations.numpy(), 0, -1)
        for i, class_name in enumerate(self._seg_class_names):  # TODO self._seg_class_names
            seg_mask = np.where(segmentations == i, np.array(1, dtype=np.ubyte), np.array(0, dtype=np.ubyte))
            self._add_segmentation_mask_for(class_name, seg_mask)

    def _process_detections(self, detections: Any, conf_thres, nms_thres, orig_image_dims) -> None:
        detections = torch.from_numpy(detections)
        detections = yoeo.utils.utils.non_max_suppression(detections, conf_thres, nms_thres)
        detections = yoeo.utils.utils.rescale_boxes(detections[0], 416, orig_image_dims).numpy()

        for detection in detections:
            c = Candidate.from_x1y1x2y2(*detection[0:4].astype(int), detection[4].astype(float))
            self._det_candidates[self._det_class_names[int(detection[5])]].append(c)  # TODO


class YOEOHandlerPytorch(YOEOHandlerTemplate):
    """
    Using Pytorch to get YOEO predictions
    """

    def __init__(self, config: Dict, model_path: str):
        super().__init__(config, model_path)
        config_path = os.path.join(model_path, "yoeo.cfg")
        weights_path = os.path.join(model_path, "yoeo.pth")

        self._model = torch_models.load_model(config_path, weights_path)

    def _compute_new_prediction(self, image, conf_thresh, nms_thres) -> None:
        detections, segmentations = torch_detect.detect_image(
            self._model,
            cv2.cvtColor(image, cv2.COLOR_BGR2RGB),
            conf_thres=conf_thresh,
            nms_thres=nms_thres
        )

        self._process_detection_output(detections)
        self._process_segmentation_output(segmentations)

    def _process_detection_output(self, detections: Any) -> None:
        for detection in detections:
            c = Candidate.from_x1y1x2y2(*detection[0:4].astype(int), detection[4].astype(float))
            self._det_candidates[self._det_class_names[int(detection[5])]].append(c)  # TODO

    def _process_segmentation_output(self, segmentations: Any) -> None:
        segmentations = np.moveaxis(segmentations, 0, -1)
        for i, class_name in enumerate(self._seg_class_names):  # TODO self._seg_class_names
            seg_mask = np.where(segmentations == i, np.array(1, dtype=np.ubyte), np.array(0, dtype=np.ubyte))
            self._add_segmentation_mask_for(class_name, seg_mask)


class YOEODetectorTemplate(CandidateFinder):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        super().__init__()

        self._yoeo_handler = yoeo_handler

    def set_image(self, image) -> None:
        self._yoeo_handler.set_image(image)

    def compute(self) -> None:
        self._yoeo_handler.predict()

    @abstractmethod
    def get_candidates(self) -> List[Candidate]:
        ...


class YOEOBallDetector(YOEODetectorTemplate):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        super().__init__(yoeo_handler)

    def get_candidates(self) -> List[Candidate]:
        return self._yoeo_handler.get_detection_candidates_for("ball")


class YOEOGoalpostDetector(YOEODetectorTemplate):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        super().__init__(yoeo_handler)

    def get_candidates(self) -> List[Candidate]:
        return self._yoeo_handler.get_detection_candidates_for("goalpost")


class YOEORobotDetector(YOEODetectorTemplate):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        super().__init__(yoeo_handler)

    def get_candidates(self) -> List[Candidate]:
        return self._yoeo_handler.get_detection_candidates_for("robot")


class IYOEOSegmentation(ABC):
    @abstractmethod
    def compute(self) -> None:
        ...

    @abstractmethod
    def get_mask(self):
        """
        Returns binary segmentation mask with values in {0, 1}
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        ...

    @abstractmethod
    def get_mask_image(self):
        """
        Returns binary segmentation mask with values in {0, 255}
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        ...

    @abstractmethod
    def set_image(self, image) -> None:
        ...


class YOEOSegmentationTemplate(IYOEOSegmentation):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        self._yoeo_handler = yoeo_handler

    def compute(self) -> None:
        self._yoeo_handler.predict()

    def get_mask_image(self):
        """
        Returns binary segmentation mask with values in {0, 255}
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        return self.get_mask() * 255

    def set_image(self, image) -> None:
        self._yoeo_handler.set_image(image)

    @abstractmethod
    def get_mask(self):
        """
        Returns binary segmentation mask with values in {0, 1}
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        ...


class YOEOBackgroundSegmentation(YOEOSegmentationTemplate):
    def __init__(self, yoeo_handler):
        super().__init__(yoeo_handler)

    def get_mask(self):
        """
        Returns binary segmentation mask with values in {0, 1}
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        return self._yoeo_handler.get_segmentation_mask_for("background")


class YOEOFieldSegmentation(YOEOSegmentationTemplate, IFieldDetector):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        super().__init__(yoeo_handler)

    def get_mask(self):
        """
        Returns binary segmentation mask with values in {0, 1}
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        return self._yoeo_handler.get_segmentation_mask_for("field")


class YOEOLineSegmentation(YOEOSegmentationTemplate):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        super().__init__(yoeo_handler)

    def get_mask(self):
        """
        Returns binary segmentation mask with values in {0, 1}
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        return self._yoeo_handler.get_segmentation_mask_for("lines")

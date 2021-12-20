from candidate import CandidateFinder, Candidate

from abc import ABC, abstractmethod
from collections import defaultdict
import cv2
from numpy import where, moveaxis
from os.path import join
from rclpy import logging
from typing import List

logger = logging.get_logger('vision_yoeo')

try:
    from yoeo import models as torch_models, detect as torch_detect
    from yoeo.utils.utils import load_classes
except ImportError:
    logger.error("Not able to import pytorchyolo. This might be fine if you use another method.")


class YOEOHandlerTemplate(ABC):
    def __init__(self, config: dict, model_path: str):
        self._use_caching = config['caching']

        self._image = None
        self._iou_non_max_suppression_thresh = config['yoeo_nms_threshold']

        self._det_candidates = defaultdict(list)
        self._det_class_names = None
        self._det_confidence_thresh = config['yoeo_conf_threshold']

        self._segmentation = None
        self._segmentations = dict()
        self._seg_class_idx = dict()

        self._load_candidate_class_names(model_path)

    def _load_candidate_class_names(self, model_path: str) -> None:
        path = join(model_path, "data", "yoeo_names.yaml")
        data = load_classes(path)

        self._det_class_names = data['detection']
        for key, i in zip(data['segmentation'], range(len(data['segmentation']))):
            self._seg_class_idx[key] = i

    def get_candidates(self, class_name: str) -> List[Candidate]:
        """
        To comply with YoloHandler interface
        """
        return self.get_detection_candidates_for(class_name)

    def get_detection_candidates_for(self, class_name: str) -> List[Candidate]:
        assert class_name in self._det_class_names, \
            f"Class '{class_name}' is not available for the current YOEO model (detection)"

        self.predict()

        return self._det_candidates[class_name]

    def predict(self) -> None:
        if self._prediction_has_to_be_updated():
            self._compute_new_prediction()

    def _prediction_has_to_be_updated(self) -> bool:
        return self._det_candidates is None or self._segmentation is None or not self._use_caching

    def get_classes(self) -> List[str]:
        """
        To comply with YoloHandler interface
        """
        return self.get_detection_candidate_class_names()

    def get_detection_candidate_class_names(self) -> List[str]:
        return self._det_class_names

    def get_segmentation_candidate_class_names(self):
        return self._seg_class_idx

    def get_segmentation_for(self, class_name: str):
        assert class_name in self._seg_class_idx, \
            f"Class '{class_name}' ist not available for the current YOEO model (segmentation)"

        self.predict()

        if class_name not in self._segmentations.keys():
            self._compute_segmentation_for(class_name)

        return self._segmentations[class_name]

    def _compute_segmentation_for(self, class_name: str) -> None:
        segmentation = where(self._segmentation == self._seg_class_idx[class_name], 1, 0)
        self._segmentations[class_name] = moveaxis(segmentation, 0, -1)

    def set_image(self, img) -> None:
        self._reset()
        self._update_image(img)

    def _reset(self) -> None:
        self._det_candidates = defaultdict(list)
        self._segmentation = None
        self._segmentations = defaultdict(list)

    def _update_image(self, img) -> None:
        self._image = img

    @abstractmethod
    def _compute_new_prediction(self) -> None:
        ...


class YOEOHandlerPytorch(YOEOHandlerTemplate):
    """
    Using Pytorch to get YOEO predictions
    """

    def __init__(self, config, model_path):
        super().__init__(config, model_path)

        weights_path = join(model_path, "weights", "yoeo.pth")
        config_path = join(model_path, "config", "yoeo-rev-7.cfg")

        self._model = torch_models.load_model(config_path, weights_path)

    def _compute_new_prediction(self) -> None:
        detections, segmentation = torch_detect.detect_image(self._model,
                                                             cv2.cvtColor(self._image, cv2.COLOR_BGR2RGB),
                                                             conf_thres=self._det_confidence_thresh,
                                                             nms_thres=self._iou_non_max_suppression_thresh)

        for detection in detections:
            c = Candidate.from_x1y1x2y2(*detection[0:4].astype(int), detection[4].astype(float))
            self._det_candidates[self._det_class_names[int(detection[5])]].append(c)

        self._segmentation = segmentation


class YOEODetector(CandidateFinder):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        super().__init__()

        self._yoeo_handler = yoeo_handler

    def set_image(self, image) -> None:
        self._yoeo_handler.set_image(image)

    @abstractmethod
    def get_candidates(self):
        ...

    def compute(self):
        self._yoeo_handler.predict()


class YOEOBallDetector(YOEODetector):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        super().__init__(yoeo_handler)

    def get_candidates(self) -> List[Candidate]:
        return self._yoeo_handler.get_detection_candidates_for("ball")


class YOEOGoalpostDetector(YOEODetector):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        super().__init__(yoeo_handler)

    def get_candidates(self):
        return self._yoeo_handler.get_detection_candidates_for("goalpost")


class YOEORobotDetector(YOEODetector):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        super().__init__(yoeo_handler)

    def get_candidates(self):
        return self._yoeo_handler.get_detection_candidates_for("robot")


class YOEOSegmentationTemplate(ABC):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        self._yoeo_handler = yoeo_handler

    @abstractmethod
    def get_segmentation(self):
        ...


class YOEOBackgroundSegmentation(YOEOSegmentationTemplate):
    def __init__(self, yoeo_handler):
        super().__init__(yoeo_handler)

    def get_segmentation(self):
        return self._yoeo_handler.get_segmentation_for("background")


class YOEOFieldSegmentation(YOEOSegmentationTemplate):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        super().__init__(yoeo_handler)

    def get_segmentation(self):
        return self._yoeo_handler.get_segmentation_for("field")


class YOEOLineSegmentation(YOEOSegmentationTemplate):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        super().__init__(yoeo_handler)

    def get_segmentation(self):
        return self._yoeo_handler.get_segmentation_for("lines")

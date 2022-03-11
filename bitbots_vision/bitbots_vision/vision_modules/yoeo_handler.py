from .candidate import CandidateFinder, Candidate
from .field_boundary import IFieldDetector

from abc import ABC, abstractmethod
from collections import defaultdict
import cv2
from numpy import where, moveaxis, array, ubyte
from os.path import join
from rclpy import logging
from typing import List, Union, Dict, Any
from yaml import load, SafeLoader

logger = logging.get_logger('vision_yoeo')

try:
    from yoeo import models as torch_models, detect as torch_detect
except ImportError:
    logger.error("Not able to import pytorchyolo. This might be fine if you use another method.")


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
    def get_segmentation_for(self, class_name: str):
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

        self._seg_candidates: Dict = dict()
        self._seg_class_names: Union[None, List[str]] = None

        self._load_candidate_class_names(model_path)

    def set_config(self, config: Dict) -> None:
        self._use_caching = config['caching']
        self._iou_non_max_suppression_thresh = config['yoeo_nms_threshold']
        self._det_confidence_thresh = config['yoeo_conf_threshold']

    def _load_candidate_class_names(self, model_path: str) -> None:
        path = join(model_path, "yoeo_names.yaml")

        with open(path, 'r', encoding="utf-8") as fp:
            class_names = load(fp, Loader=SafeLoader)

        assert "detection" in class_names.keys(), f"Missing key 'detection' in {path}"
        assert "segmentation" in class_names.keys(), f"Missing key 'segmentation' in {path}"

        self._det_class_names = class_names['detection']
        self._seg_class_names = class_names['segmentation']

    def get_detection_candidates_for(self, class_name: str) -> List[Candidate]:
        assert class_name in self._det_class_names, \
            f"Class '{class_name}' is not available for the current YOEO model (detection)"

        self.predict()

        return self._det_candidates[class_name]

    def predict(self) -> None:
        if self._prediction_has_to_be_updated():
            self._compute_new_prediction()

    def _prediction_has_to_be_updated(self) -> bool:
        return not self._use_caching or not (self._seg_candidates or self._det_candidates)

    def get_detection_candidate_class_names(self) -> List[str]:
        return self._det_class_names

    def get_segmentation_candidate_class_names(self) -> List[str]:
        return self._seg_class_names

    def get_segmentation_for(self, class_name: str):
        """
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        assert class_name in self._seg_class_names, \
            f"Class '{class_name}' ist not available for the current YOEO model (segmentation)"

        self.predict()

        return self._seg_candidates[class_name]

    def set_image(self, img) -> None:
        self._reset()
        self._update_image(img)

    def _reset(self) -> None:
        self._det_candidates = defaultdict(list)
        self._seg_candidates = dict()

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
        weights_path = join(model_path, "yoeo.pth")
        config_path = join(model_path, "yoeo.cfg")

        self._model = torch_models.load_model(config_path, weights_path)

    def _compute_new_prediction(self) -> None:
        detections, segmentation = torch_detect.detect_image(self._model,
                                                             cv2.cvtColor(self._image, cv2.COLOR_BGR2RGB),
                                                             conf_thres=self._det_confidence_thresh,
                                                             nms_thres=self._iou_non_max_suppression_thresh)
        self._set_detection_candidates(detections)
        self._set_segmentations(segmentation)

    def _set_detection_candidates(self, detections: Any) -> None:
        for detection in detections:
            c = Candidate.from_x1y1x2y2(*detection[0:4].astype(int), detection[4].astype(float))
            self._det_candidates[self._det_class_names[int(detection[5])]].append(c)

    def _set_segmentations(self, segmentation: Any) -> None:
        segmentation = moveaxis(segmentation, 0, -1)
        for i, class_name in enumerate(self._seg_class_names):
            seg = where(segmentation == i, array(1, dtype=ubyte), array(0, dtype=ubyte))
            self._seg_candidates[class_name] = seg


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

    def set_image(self, image) -> None:
        self._yoeo_handler.set_image(image)

    @abstractmethod
    def get_mask(self):
        """
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        ...


class YOEOBackgroundSegmentation(YOEOSegmentationTemplate):
    def __init__(self, yoeo_handler):
        super().__init__(yoeo_handler)

    def get_mask(self):
        """
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        return self._yoeo_handler.get_segmentation_for("background")


class YOEOFieldSegmentation(YOEOSegmentationTemplate, IFieldDetector):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        super().__init__(yoeo_handler)

    def get_mask(self):
        """
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        return self._yoeo_handler.get_segmentation_for("field")

    def get_mask_image(self):
        """
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        return self.get_mask() * 255


class YOEOLineSegmentation(YOEOSegmentationTemplate):
    def __init__(self, yoeo_handler: YOEOHandlerTemplate):
        super().__init__(yoeo_handler)

    def get_mask(self):
        """
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        return self._yoeo_handler.get_segmentation_for("lines")

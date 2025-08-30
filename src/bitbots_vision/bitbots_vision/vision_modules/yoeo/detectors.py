from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from bitbots_vision.vision_modules.candidate import Candidate, CandidateFinder

from . import yoeo_handlers


class DetectorTemplate(CandidateFinder):
    """
    Abstract base class for YOEO detectors. Actual detectors only need to implement the abstract method get_candidates()
    if they inherit from this template.
    """

    def __init__(self, yoeo_handler: yoeo_handlers.IYOEOHandler):
        super().__init__()

        self._yoeo_handler = yoeo_handler

    def set_image(self, image: np.ndarray) -> None:
        """
        A subsequent call to compute() will run the YOEO network on this image.

        :param image: the image to run the YOEO network on
        :type image: np.ndarray
        """

        self._yoeo_handler.set_image(image)

    def compute(self) -> None:
        """
        Runs the YOEO network (if necessary) on the current input image.
        """

        self._yoeo_handler.predict()

    @abstractmethod
    def get_candidates(self) -> list[Candidate]:
        """
        Returns the detection candidates.
        """
        ...


class BallDetector(DetectorTemplate):
    """
    YOEO Ball Detection class.
    """

    def __init__(self, yoeo_handler: yoeo_handlers.IYOEOHandler):
        super().__init__(yoeo_handler)

    def get_candidates(self) -> list[Candidate]:
        return self._yoeo_handler.get_detection_candidates_for("ball")


class GoalpostDetector(DetectorTemplate):
    """
    YOEO Goalpost Detection class.
    """

    def __init__(self, yoeo_handler: yoeo_handlers.IYOEOHandler):
        super().__init__(yoeo_handler)

    def get_candidates(self) -> list[Candidate]:
        return self._yoeo_handler.get_detection_candidates_for("goalpost")


class RobotDetector(DetectorTemplate):
    """
    YOEO Robot Detection class.
    """

    def __init__(self, yoeo_handler: yoeo_handlers.IYOEOHandler):
        super().__init__(yoeo_handler)

    def get_candidates(self) -> list[Candidate]:
        return self._yoeo_handler.get_detection_candidates_for("robot")


class BlueRobotDetector(DetectorTemplate):
    """
    YOEO Robot Detection class for blue robots.
    """

    def __init__(self, yoeo_handler: yoeo_handlers.IYOEOHandler):
        super().__init__(yoeo_handler)

    def get_candidates(self) -> list[Candidate]:
        return self._yoeo_handler.get_detection_candidates_for("robot_blue")


class RedRobotDetector(DetectorTemplate):
    """
    YOEO Robot Detection class for red robots.
    """

    def __init__(self, yoeo_handler: yoeo_handlers.IYOEOHandler):
        super().__init__(yoeo_handler)

    def get_candidates(self) -> list[Candidate]:
        return self._yoeo_handler.get_detection_candidates_for("robot_red")


class UnknownRobotDetector(DetectorTemplate):
    """
    YOEO Robot Detection class for unknown robots.
    """

    def __init__(self, yoeo_handler: yoeo_handlers.IYOEOHandler):
        super().__init__(yoeo_handler)

    def get_candidates(self) -> list[Candidate]:
        return self._yoeo_handler.get_detection_candidates_for("robot_unknown")


class ISegmentation(ABC):
    """
    Interface of YOEO segmentation classes.
    """

    @abstractmethod
    def compute(self) -> None:
        """
        Runs the YOEO network (if necessary) on the current input image.
        """
        ...

    @abstractmethod
    def get_mask(self) -> np.ndarray:
        """
        Returns binary segmentation mask with values in {0, 1}.

        :return: binary segmentation mask
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        ...

    @abstractmethod
    def get_mask_image(self) -> np.ndarray:
        """
        Returns binary segmentation mask with values in {0, 255}.

        :return: binary segmentation mask
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        ...

    @abstractmethod
    def set_image(self, image: np.ndarray) -> None:
        """
        A subsequent call to compute() will run the YOEO network on this image.

        :param image: the image to run the YOEO network on
        :type image: np.ndarray
        """
        ...


class SegmentationTemplate(ISegmentation):
    """
    Abstract base implementation of the IYOEOSegmentation interface. Actual classes need only implement the abstract
    method get_mask() if they inherit from this template.
    """

    def __init__(self, yoeo_handler: yoeo_handlers.IYOEOHandler):
        self._yoeo_handler = yoeo_handler

    def compute(self) -> None:
        self._yoeo_handler.predict()

    def get_mask_image(self):
        return self.get_mask() * 255

    def set_image(self, image) -> None:
        self._yoeo_handler.set_image(image)

    @abstractmethod
    def get_mask(self): ...


class BackgroundSegmentation(SegmentationTemplate):
    """
    YOEO Background Segmentation class.
    """

    def __init__(self, yoeo_handler: yoeo_handlers.IYOEOHandler):
        super().__init__(yoeo_handler)

    def get_mask(self):
        return self._yoeo_handler.get_segmentation_mask_for("background")


class FieldSegmentation(SegmentationTemplate):
    """
    YOEO Field Segmentation class.
    """

    def __init__(self, yoeo_handler: yoeo_handlers.IYOEOHandler):
        super().__init__(yoeo_handler)

    def get_mask(self):
        return self._yoeo_handler.get_segmentation_mask_for("field")


class LineSegmentation(SegmentationTemplate):
    """
    YOEO Line Segmentation class.
    """

    def __init__(self, yoeo_handler: yoeo_handlers.IYOEOHandler):
        super().__init__(yoeo_handler)

    def get_mask(self):
        return self._yoeo_handler.get_segmentation_mask_for("lines")

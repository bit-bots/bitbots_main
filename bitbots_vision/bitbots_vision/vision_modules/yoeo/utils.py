from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from jaxtyping import UInt8

logger = rclpy.logging.get_logger("yoeo_handler_utils")


@dataclass
class ImagePreProcessorData:
    """
    This dataclass is used to exchange relevant parameters of the applied image preprocessing between instances of type
    IImagePreProcessor and instances of type ISegmentationPostProcessor and IDetectionPostProcessor, respectively.
    :param padding_top: applied padding (number of pixels) at the top of the image
    :type padding_top: int
    :param padding_bottom: applied padding (number of pixels) at the bottom of the image
    :type padding_bottom: int
    :param padding_left: applied padding (number of pixels) at the left-hand side of the image
    :type padding_left: int
    :param padding_right: applied padding (number of pixels) at the right-hand side of the image
    :type padding_right: int
    :param max_dim: the larger of the two dimensions of the original unprocessed image (number of pixels)
    :type max_dim: int
    """

    padding_top: int
    padding_bottom: int
    padding_left: int
    padding_right: int
    max_dim: int


class IImagePreProcessor(ABC):
    """
    Interface for YOEO image pre-processors. Implementing classes should take an ordinary image as input and output it
    in such a way that it can be input into the YOEO network.
    """

    @abstractmethod
    def configure(self, network_input_shape: Tuple[int, int]) -> None:
        """
        Allows to (re-) configure the current instance.

        :param network_input_shape: input shape of the YOEO network (height, width)
        :type network_input_shape: Tuple[int, int]
        """
        ...

    @abstractmethod
    def get_info(self) -> ImagePreProcessorData:
        """
        Returns relevant image pre-processing parameters that are needed by the respective image post-processors.

        Invoking this method before an image has been pre-processed returns an ImagePreProcessorData object with
        default values.
        """
        ...

    @abstractmethod
    def process(self, image: np.ndarray) -> np.ndarray:
        """
        Run the image pre-processing on the method's argument.

        :param image: the image to pre-process (axis order: height, width, channels)
        :type image: np.ndarray
        :return: the pre-processed image  (axis order: channels, height, width)
        :rtype: np.ndarray
        """
        ...

    @abstractmethod
    def reset(self) -> None:
        """
        Resets the instance into a state as if no image has been processed yet.
        """
        ...


class ISegmentationPostProcessor:
    """
    Interface for YOEO segmentation post-processors. Implementing classes should take a YOEO segmentation network
    output as input and output a segmentation with size equal to the original image.
    """

    @abstractmethod
    def configure(self, image_preprocessor: IImagePreProcessor) -> None:
        """
        Allows to (re-) configure the current instance.

        :param image_preprocessor: instance of IImagePreProcessor that handles the image pre-processing
        :type image_preprocessor: IImagePreProcessor
        """
        ...

    @abstractmethod
    def process(self, segmentation: np.ndarray) -> np.ndarray:
        """
        Run the segmentation post-processing on the method's argument.

        :param segmentation: YOEO segmentation network output (axis order: channels, height, width)
        :type segmentation: np.ndarray
        :return: the post-processed segmentation output (axis order: height, width)
        :rtype: np.ndarray
        """
        ...


class IDetectionPostProcessor:
    """
    Interface for YOEO detection post-processors. Implementing classes should take a YOEO detection network
    output as input, perform non-maximum suppression on it and rescale the bounding boxes to the original image size.
    """

    @abstractmethod
    def configure(
        self,
        image_preprocessor: IImagePreProcessor,
        output_img_size: int,
        conf_thresh: float,
        nms_thresh: float,
        robot_class_ids: List[int],
    ) -> None:
        """
        Allows to (re-) configure the current instance.

        :param image_preprocessor: instance of IImagePreProcessor that handles the image pre-processing
        :type image_preprocessor: IImagePreProcessor
        :param output_img_size: image size (1D) for which the detections are calculated by the YOEO network
        :type output_img_size: int
        :param conf_thresh: class confidence threshold used in non-maximum suppression
        :type conf_thresh: float
        :param nms_thresh: threshold used in non-maximum suppression
        :type nms_thresh: float
        :param robot_class_ids: class ids of robot classes (required for nms across all robot classes)
        :type robot_class_ids: List[int]
        """
        ...

    @abstractmethod
    def process(self, detections: np.ndarray) -> np.ndarray:
        """
        Run the detection post-processing on the method's argument.

        :param detections: YOEO detection network output (axis layout: 1, number of boxes, boxes),
                           (boxes layout: x, y, w, h, obj_conf, cond_class_conf_1, cond_class_conf_2, ...)
        :type detections: np.ndarray
        :return: the post-processed detection output
        :rtype: np.ndarray
        """
        ...


class DefaultImagePreProcessor(IImagePreProcessor):
    def __init__(self, network_input_shape):
        self._network_input_shape_WH: Optional[Tuple[int, int]] = None  # (width, height)
        self.configure(network_input_shape)

        # these attributes change for every image!
        self._image_dimensions_HW: Tuple[int, int] = (0, 0)  # (height, width)
        self._padding_top: int = 0
        self._padding_bottom: int = 0
        self._padding_left: int = 0
        self._padding_right: int = 0

    def configure(self, network_input_shape: Tuple[int, int]) -> None:
        # change layout from (height, width) to (width, height)
        self._network_input_shape_WH = network_input_shape[::-1]

    def get_info(self) -> ImagePreProcessorData:
        return ImagePreProcessorData(
            padding_top=self._padding_top,
            padding_bottom=self._padding_bottom,
            padding_left=self._padding_left,
            padding_right=self._padding_right,
            max_dim=np.max(self._image_dimensions_HW),
        )

    def process(self, image: UInt8[np.ndarray, "h w 3"]) -> UInt8[np.ndarray, "3 network_input_h network_input_w"]:
        self._image_dimensions_HW = image.shape[:2]  # type: ignore[assignment]
        self._calculate_paddings()

        image = self._normalize_image_to_range_0_1(image)
        image = self._pad_to_square(image)
        image = self._resize_to_network_input_shape(image)
        image = self._rearrange_axes_from_hwc_to_chw(image)

        return image

    def _calculate_paddings(self) -> None:
        height, width = self._image_dimensions_HW

        total_vertical_padding = max(0, width - height)
        total_horizontal_padding = max(0, height - width)

        self._padding_top = total_vertical_padding // 2
        self._padding_bottom = total_vertical_padding - self._padding_top
        self._padding_left = total_horizontal_padding // 2
        self._padding_right = total_horizontal_padding - self._padding_left

    @staticmethod
    def _normalize_image_to_range_0_1(image: np.ndarray) -> np.ndarray:
        return image / 255

    def _pad_to_square(self, image: np.ndarray) -> np.ndarray:
        return cv2.copyMakeBorder(
            src=image,
            top=self._padding_top,
            bottom=self._padding_bottom,
            left=self._padding_left,
            right=self._padding_right,
            borderType=cv2.BORDER_CONSTANT,
            value=0,
        )  # type: ignore[call-overload]

    def _resize_to_network_input_shape(self, image: np.ndarray) -> np.ndarray:
        return cv2.resize(src=image, dsize=self._network_input_shape_WH)

    @staticmethod
    def _rearrange_axes_from_hwc_to_chw(image: np.ndarray) -> np.ndarray:
        # Change data layout from HWC to CHW
        return np.moveaxis(image, 2, 0)

    def reset(self) -> None:
        self._image_dimensions_HW = (0, 0)
        self._padding_top = 0
        self._padding_bottom = 0
        self._padding_left = 0
        self._padding_right = 0


class DefaultSegmentationPostProcessor(ISegmentationPostProcessor):
    def __init__(self, image_preprocessor: IImagePreProcessor):
        self._image_preprocessor = image_preprocessor

        # these attributes change for every segmentation!
        self._max_dim: int = 0
        self._padding_top: int = 0
        self._padding_bottom: int = 0
        self._padding_left: int = 0
        self._padding_right: int = 0

    def configure(self, image_preprocessor: IImagePreProcessor) -> None:
        self._image_preprocessor = image_preprocessor

    def process(self, segmentation: np.ndarray) -> np.ndarray:
        self._get_preprocessor_info()
        segmentation = self._rearrange_axes_from_chw_to_hwc(segmentation)
        segmentation = self._resize_to_original_padded_size(segmentation)
        segmentation = self._unpad(segmentation)

        return segmentation

    def _get_preprocessor_info(self) -> None:
        preprocessor_info = self._image_preprocessor.get_info()
        self._max_dim = preprocessor_info.max_dim
        self._padding_top = preprocessor_info.padding_top
        self._padding_bottom = preprocessor_info.padding_bottom
        self._padding_left = preprocessor_info.padding_left
        self._padding_right = preprocessor_info.padding_right

    @staticmethod
    def _rearrange_axes_from_chw_to_hwc(segmentation: np.ndarray) -> np.ndarray:
        # Change data layout from CHW to HWC
        return np.moveaxis(segmentation, 0, -1)

    def _resize_to_original_padded_size(self, segmentation: np.ndarray) -> np.ndarray:
        return cv2.resize(src=segmentation, dsize=(self._max_dim, self._max_dim), interpolation=cv2.INTER_NEAREST_EXACT)

    def _unpad(self, segmentation: np.ndarray) -> np.ndarray:
        return segmentation[
            self._padding_top : self._max_dim - self._padding_bottom,
            self._padding_left : self._max_dim - self._padding_right,
            ...,
        ]


class DefaultDetectionPostProcessor(IDetectionPostProcessor):
    def __init__(
        self,
        image_preprocessor: IImagePreProcessor,
        output_img_size: int,
        conf_thresh: float,
        nms_thresh: float,
        robot_class_ids: List[int],
    ):
        self._image_preprocessor: IImagePreProcessor = image_preprocessor
        self._output_img_size: int = output_img_size
        self._conf_thresh: float = conf_thresh
        self._nms_thresh: float = nms_thresh

        # These values are needed in order to perform a proper nms if multiple robot classes exist,
        # i. e. if nms shall be performed across all robot classes and not per robot class
        self._robot_class_ids = robot_class_ids

        self._nms_max_number_of_boxes = 30000
        self._nms_max_number_of_detections_per_image = 30
        self._nms_max_width_height_in_pixels = 4096

        # these attributes change for every segmentation!
        self._max_dim: int = 0
        self._padding_top: int = 0
        self._padding_bottom: int = 0
        self._padding_left: int = 0
        self._padding_right: int = 0

    def configure(
        self,
        image_preprocessor: IImagePreProcessor,
        output_img_size: int,
        conf_thresh: float,
        nms_thresh: float,
        robot_class_ids: List[int],
    ) -> None:
        self._image_preprocessor = image_preprocessor
        self._output_img_size = output_img_size
        self._conf_thresh = conf_thresh
        self._nms_thresh = nms_thresh
        self._robot_class_ids = robot_class_ids

    def process(self, detections: np.ndarray) -> np.ndarray:
        self._get_preprocessor_info()
        detections = self._perform_nms(detections[0, ...])
        detections = self._rescale_boxes(detections)

        return detections

    def _get_preprocessor_info(self) -> None:
        preprocessor_info = self._image_preprocessor.get_info()
        self._max_dim = preprocessor_info.max_dim
        self._padding_top = preprocessor_info.padding_top
        self._padding_bottom = preprocessor_info.padding_bottom
        self._padding_left = preprocessor_info.padding_left
        self._padding_right = preprocessor_info.padding_right

    def _perform_nms(self, detections: np.ndarray) -> np.ndarray:
        detections = self._preprocess_detections_for_nms(detections)

        boxes, scores = self._get_boxes_and_scores_for_nms(detections)
        indices = cv2.dnn.NMSBoxes(
            bboxes=boxes,
            scores=scores,
            score_threshold=0,
            nms_threshold=self._nms_thresh,
            top_k=self._nms_max_number_of_detections_per_image,
        )

        output = self._postprocess_nms_output(detections, indices)

        return output

    def _preprocess_detections_for_nms(self, detections: np.ndarray) -> np.ndarray:
        detections = self._filter_by_objectness_confidence(detections)
        detections = self._calculate_class_confidence_scores(detections)
        detections = self._pin_down_last_dimension_to_6(detections)

        return detections

    def _filter_by_objectness_confidence(self, detections: np.ndarray) -> np.ndarray:
        return detections[detections[:, 4] > self._conf_thresh]

    @staticmethod
    def _calculate_class_confidence_scores(detections: np.ndarray) -> np.ndarray:
        # class_confidence_score = conditional_class_probability * box_confidence_score (objectness)
        # p(class, object)       = p(class | object)             * p(object)
        detections[:, 5:] *= detections[:, 4:5]

        return detections

    def _pin_down_last_dimension_to_6(self, detections: np.ndarray) -> np.ndarray:
        box_coordinates = detections[:, :4]
        class_confidence_scores = detections[:, 5:]

        i, j = (class_confidence_scores > self._conf_thresh).nonzero()
        x = np.concatenate((box_coordinates[i], class_confidence_scores[i, j, None], j[:, None]), axis=1)

        return x

    def _get_boxes_and_scores_for_nms(self, detections: np.ndarray) -> Tuple:
        if self._robot_class_ids:
            class_offsets = np.where(
                self._is_robot_class(detections[:, 5:6]), self._robot_class_ids[0], detections[:, 5:6]
            )
        else:
            class_offsets = detections[:, 5:6].copy()

        class_offsets *= self._nms_max_width_height_in_pixels

        box_coords = detections[:, :4].copy()
        box_coords[:, :2] += class_offsets

        return box_coords, detections[:, 4]

    def _is_robot_class(self, detections: np.ndarray) -> np.ndarray:
        return np.isin(detections, self._robot_class_ids)

    def _postprocess_nms_output(self, detections: np.ndarray, nms_indices) -> np.ndarray:
        detections = detections[nms_indices]
        detections = self._convert_box_coordinates(detections)

        return detections

    @staticmethod
    def _convert_box_coordinates(boxes: np.ndarray) -> np.ndarray:
        """
        Transform bounding box coordinates from xywh format (centroid coordinates, width, height) to xyxy format (upper
        left coordinates, lower right coordinates)
        :param boxes: boxes with coordinates in xywh format (axis layout: number of boxes, boxes)
        :type boxes: np.ndarray
        :return: boxes with coordinates in xyxy format (axis layout: number of boxes, boxes)
        :rtype: np.ndarray
        """
        coordinates_xyxy = np.zeros_like(boxes[:, :4])
        coordinates_xyxy[..., 0] = boxes[..., 0] - boxes[..., 2] / 2
        coordinates_xyxy[..., 1] = boxes[..., 1] - boxes[..., 3] / 2
        coordinates_xyxy[..., 2] = boxes[..., 0] + boxes[..., 2] / 2
        coordinates_xyxy[..., 3] = boxes[..., 1] + boxes[..., 3] / 2
        boxes[:, :4] = coordinates_xyxy

        return boxes

    def _rescale_boxes(self, boxes):
        rescaled_boxes = self._rescale_boxes_to_original_padded_img_size(boxes)
        rescaled_boxes = self._unpad_box_coordinates(rescaled_boxes)

        return rescaled_boxes

    def _rescale_boxes_to_original_padded_img_size(self, boxes):
        scale_factor = self._max_dim / self._output_img_size
        boxes[:, 0:4] = boxes[:, 0:4] * scale_factor

        return boxes

    def _unpad_box_coordinates(self, boxes):
        boxes[:, 0] = boxes[:, 0] - self._padding_left
        boxes[:, 1] = boxes[:, 1] - self._padding_top
        boxes[:, 2] = boxes[:, 2] - self._padding_left
        boxes[:, 3] = boxes[:, 3] - self._padding_top

        return boxes

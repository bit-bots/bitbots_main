from dataclasses import dataclass
from abc import ABC, abstractmethod
import numpy as np
import cv2
import rclpy
import yoeo.utils.utils
from typing import Tuple, Optional

logger = rclpy.logging.get_logger('yoeo_handler_utils')


@dataclass
class ImagePreprocessorData:
    applied_padding_h: int
    applied_padding_w: int
    max_dim: int


class IImagePreprocessor(ABC):
    @abstractmethod
    def get_info(self) -> ImagePreprocessorData:
        ...

    @abstractmethod
    def process(self, image):
        ...

    @abstractmethod
    def reset(self) -> None:
        ...

    @abstractmethod
    def configure(self, network_input_shape: Tuple[int, int]) -> None:
        ...


class ISegmentationPostProcessor:
    @abstractmethod
    def process(self, segmentation):
        ...

    @abstractmethod
    def configure(self, image_preprocessor: IImagePreprocessor) -> None:
        ...


class IDetectionPostProcessor:
    @abstractmethod
    def process(self, detections):
        ...

    @abstractmethod
    def configure(self, image_preprocessor: IImagePreprocessor, conf_thresh: float, nms_thresh: float) -> None:
        ...


class OVImagePreprocessor(IImagePreprocessor):
    def __init__(self, network_input_shape):
        self._network_input_shape_WH: Optional[Tuple[int, int]] = None
        self.configure(network_input_shape)

        # these attributes change for every image!
        self._image_dimensions: Tuple[int, int] = (0, 0)  # (height, width)
        self._applied_padding_h: int = 0
        self._applied_padding_w: int = 0

    def get_info(self) -> ImagePreprocessorData:
        return ImagePreprocessorData(
            applied_padding_h=self._applied_padding_h,
            applied_padding_w=self._applied_padding_w,
            max_dim=np.max(self._image_dimensions)
        )

    def process(self, image):
        self._image_dimensions = image.shape[:2]
        self._calculate_paddings()

        image = self._normalize_image_to_range_0_1(image)
        image = self._pad_to_square(image)
        image = self._resize_to_network_input_shape(image)
        image = self._rearrange_axes_from_HWC_to_CHW(image)

        return image

    def _calculate_paddings(self) -> None:
        max_dim = np.max(self._image_dimensions)
        height, width = self._image_dimensions
        self._applied_padding_h = (max_dim - height) // 2
        self._applied_padding_w = (max_dim - width) // 2

    @staticmethod
    def _normalize_image_to_range_0_1(image):
        return image.astype(np.float64) / 255  # TODO müsste auch ohne astype gehen

    def _pad_to_square(self, image):
        return cv2.copyMakeBorder(
            src=image,
            top=self._applied_padding_h,
            bottom=self._applied_padding_h,
            left=self._applied_padding_w,
            right=self._applied_padding_w,
            borderType=cv2.BORDER_CONSTANT,
            value=0
        )

    def _resize_to_network_input_shape(self, image):
        return cv2.resize(src=image, dsize=self._network_input_shape_WH)

    @staticmethod
    def _rearrange_axes_from_HWC_to_CHW(image):
        # Change data layout from HWC to CHW
        return np.moveaxis(image, 2, 0)

    def reset(self) -> None:
        self._image_dimensions = (0, 0)
        self._applied_padding_h = 0
        self._applied_padding_w = 0

    def configure(self, network_input_shape: Tuple[int, int]) -> None:
        self._network_input_shape_WH = network_input_shape[::-1]  # (height, width) to (width, height)


class ONNXImagePreprocessor(IImagePreprocessor):
    def __init__(self, network_input_dimensions):
        self._image_prepocessor: IImagePreprocessor = OVImagePreprocessor(network_input_dimensions)

    def get_info(self) -> ImagePreprocessorData:
        return self._image_prepocessor.get_info()

    def process(self, image):
        return self._image_prepocessor.process(image)

    def reset(self) -> None:
        self._image_prepocessor.reset()

    def configure(self, network_input_shape: Tuple[int, int]) -> None:
        self._image_prepocessor.configure(network_input_shape)


class OVSegmentationPostProcessor(ISegmentationPostProcessor):
    def __init__(self, image_preprocessor: IImagePreprocessor):
        self._image_preprocessor = image_preprocessor

        # these attributes change for every segmentation!
        self._applied_padding_h: int = 0
        self._applied_padding_w: int = 0
        self._max_dim: int = 0

    def configure(self, image_preprocessor: IImagePreprocessor) -> None:
        self._image_preprocessor = image_preprocessor

    def process(self, segmentation):
        self._get_preprocessor_info()
        segmentation = self._rearrange_axis_from_CHW_to_HWC(segmentation)
        segmentation = self._resize_to_original_size(segmentation)
        segmentation = self._remove_padding(segmentation)
        return segmentation

    def _get_preprocessor_info(self) -> None:
        preprocessor_info = self._image_preprocessor.get_info()
        self._max_dim = preprocessor_info.max_dim
        self._applied_padding_h = preprocessor_info.applied_padding_h
        self._applied_padding_w = preprocessor_info.applied_padding_w

    @staticmethod
    def _change_dtype_from_int64_to_uint8(image):
        return image.astype(np.uint8)

    @staticmethod
    def _rearrange_axis_from_CHW_to_HWC(segmentation):
        # Change data layout from CHW to HWC
        return np.moveaxis(segmentation, 0, -1)

    def _resize_to_original_size(self, segmentation):
        return cv2.resize(
            src=segmentation,
            dsize=(self._max_dim, self._max_dim),
            interpolation=cv2.INTER_NEAREST_EXACT
        )

    def _remove_padding(self, segmentation):
        return segmentation[self._applied_padding_h:self._max_dim - self._applied_padding_h,
               self._applied_padding_w:self._max_dim - self._applied_padding_w, ...]


class ONNXSegmentationPostProcessor(ISegmentationPostProcessor):
    def __init__(self, image_preprocessor: IImagePreprocessor):
        self._seg_postprocessor: ISegmentationPostProcessor = OVSegmentationPostProcessor(image_preprocessor)

    def configure(self, image_preprocessor: IImagePreprocessor) -> None:
        self._seg_postprocessor.configure(image_preprocessor)

    def process(self, segmentation):
        return self._seg_postprocessor.process(segmentation)


class OVDetectionPostProcessor(IDetectionPostProcessor):
    def __init__(self, image_preprocessor: IImagePreprocessor, conf_thresh: float, nms_thresh: float):
        self._image_preprocessor: IImagePreprocessor = image_preprocessor
        self._conf_thresh: float = conf_thresh
        self._nms_thresh: float = nms_thresh

    def process(self, detections):
        import torch
        import yoeo
        detections = torch.from_numpy(detections)
        detections = yoeo.utils.utils.non_max_suppression(detections, self._conf_thresh, self._nms_thresh)
        original_dims = (
            self._image_preprocessor.get_info().max_dim - 2 * self._image_preprocessor.get_info().applied_padding_h,
            self._image_preprocessor.get_info().max_dim - 2 * self._image_preprocessor.get_info().applied_padding_w)
        detections = yoeo.utils.utils.rescale_boxes(detections[0], 416, original_dims).numpy()
        return detections

    def configure(self, image_preprocessor: IImagePreprocessor, conf_thresh: float, nms_thresh: float) -> None:
        self._image_preprocessor = image_preprocessor
        self._conf_thresh = conf_thresh
        self._nms_thresh = nms_thresh


class ONNXDetectionPostProcessor(IDetectionPostProcessor):
    def __init__(self, image_preprocessor: IImagePreprocessor, conf_thresh: float, nms_thresh: float):
        self._det_postprocessor: IDetectionPostProcessor = OVDetectionPostProcessor(
            image_preprocessor=image_preprocessor,
            conf_thresh=conf_thresh,
            nms_thresh=nms_thresh
        )

    def process(self, detections):
        return self._det_postprocessor.process(detections)

    def configure(self, image_preprocessor: IImagePreprocessor, conf_thresh: float, nms_thresh: float) -> None:
        self._det_postprocessor.configure(
            image_preprocessor=image_preprocessor,
            conf_thresh=conf_thresh,
            nms_thresh=nms_thresh
        )
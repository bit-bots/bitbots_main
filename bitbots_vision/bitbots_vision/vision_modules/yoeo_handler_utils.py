from dataclasses import dataclass
from abc import ABC, abstractmethod
import numpy as np
import cv2
import rclpy
from typing import Tuple, Optional

logger = rclpy.logging.get_logger('yoeo_handler_utils')


@dataclass
class ImagePreProcessorData:
    padding_top: int
    padding_bottom: int
    padding_left: int
    padding_right: int
    max_dim: int


class IImagePreProcessor(ABC):
    @abstractmethod
    def configure(self, network_input_shape: Tuple[int, int]) -> None:
        ...

    @abstractmethod
    def get_info(self) -> ImagePreProcessorData:
        ...

    @abstractmethod
    def process(self, image):
        ...

    @abstractmethod
    def reset(self) -> None:
        ...


class ISegmentationPostProcessor:
    @abstractmethod
    def configure(self, image_preprocessor: IImagePreProcessor) -> None:
        ...

    @abstractmethod
    def process(self, segmentation):
        ...


class IDetectionPostProcessor:
    @abstractmethod
    def configure(self,
                  image_preprocessor: IImagePreProcessor,
                  output_img_size: int,
                  conf_thresh: float,
                  nms_thresh: float) -> None:
        ...

    @abstractmethod
    def process(self, detections):
        ...


class OVImagePreProcessor(IImagePreProcessor):
    def __init__(self, network_input_shape):
        self._network_input_shape_WH: Optional[Tuple[int, int]] = None
        self.configure(network_input_shape)

        # these attributes change for every image!
        self._image_dimensions: Tuple[int, int] = (0, 0)  # (height, width)
        self._padding_top: int = 0
        self._padding_bottom: int = 0
        self._padding_left: int = 0
        self._padding_right: int = 0

    def configure(self, network_input_shape: Tuple[int, int]) -> None:
        self._network_input_shape_WH = network_input_shape[::-1]  # (height, width) to (width, height)

    def get_info(self) -> ImagePreProcessorData:
        return ImagePreProcessorData(
            padding_top=self._padding_top,
            padding_bottom=self._padding_bottom,
            padding_left=self._padding_left,
            padding_right=self._padding_right,
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
        height, width = self._image_dimensions

        total_vertical_padding = max(0, width - height)
        total_horizontal_padding = max(0, height - width)

        self._padding_top = total_vertical_padding // 2
        self._padding_bottom = total_vertical_padding - self._padding_top
        self._padding_left = total_horizontal_padding // 2
        self._padding_right = total_horizontal_padding - self._padding_left

    @staticmethod
    def _normalize_image_to_range_0_1(image):
        return image/ 255

    def _pad_to_square(self, image):
        return cv2.copyMakeBorder(
            src=image,
            top=self._padding_top,
            bottom=self._padding_bottom,
            left=self._padding_left,
            right=self._padding_right,
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
        self._padding_top = 0
        self._padding_bottom = 0
        self._padding_left = 0
        self._padding_right = 0


class ONNXImagePreProcessor(IImagePreProcessor):
    def __init__(self, network_input_dimensions):
        self._image_prepocessor: IImagePreProcessor = OVImagePreProcessor(network_input_dimensions)

    def configure(self, network_input_shape: Tuple[int, int]) -> None:
        self._image_prepocessor.configure(network_input_shape)

    def get_info(self) -> ImagePreProcessorData:
        return self._image_prepocessor.get_info()

    def process(self, image):
        return self._image_prepocessor.process(image)

    def reset(self) -> None:
        self._image_prepocessor.reset()


class TVMImagePreProcessor(IImagePreProcessor):
    def __init__(self, network_input_dimensions):
        self._image_prepocessor: IImagePreProcessor = OVImagePreProcessor(network_input_dimensions)

    def configure(self, network_input_shape: Tuple[int, int]) -> None:
        self._image_prepocessor.configure(network_input_shape)

    def get_info(self) -> ImagePreProcessorData:
        return self._image_prepocessor.get_info()

    def process(self, image):
        return self._image_prepocessor.process(image)

    def reset(self) -> None:
        self._image_prepocessor.reset()


class OVSegmentationPostProcessor(ISegmentationPostProcessor):
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

    def process(self, segmentation):
        self._get_preprocessor_info()
        segmentation = self._rearrange_axis_from_CHW_to_HWC(segmentation)
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
    def _change_dtype_from_int64_to_uint8(image):
        return image.astype(np.uint8)

    @staticmethod
    def _rearrange_axis_from_CHW_to_HWC(segmentation):
        # Change data layout from CHW to HWC
        return np.moveaxis(segmentation, 0, -1)

    def _resize_to_original_padded_size(self, segmentation):
        return cv2.resize(
            src=segmentation,
            dsize=(self._max_dim, self._max_dim),
            interpolation=cv2.INTER_NEAREST_EXACT
        )

    def _unpad(self, segmentation):
        return segmentation[self._padding_top:self._max_dim - self._padding_bottom,
                            self._padding_left:self._max_dim - self._padding_right, ...]


class ONNXSegmentationPostProcessor(ISegmentationPostProcessor):
    def __init__(self, image_preprocessor: IImagePreProcessor):
        self._seg_postprocessor: ISegmentationPostProcessor = OVSegmentationPostProcessor(image_preprocessor)

    def configure(self, image_preprocessor: IImagePreProcessor) -> None:
        self._seg_postprocessor.configure(image_preprocessor)

    def process(self, segmentation):
        return self._seg_postprocessor.process(segmentation)


class TVMSegmentationPostProcessor(ISegmentationPostProcessor):
    def __init__(self, image_preprocessor: IImagePreProcessor):
        self._seg_postprocessor: ISegmentationPostProcessor = OVSegmentationPostProcessor(image_preprocessor)

    def configure(self, image_preprocessor: IImagePreProcessor) -> None:
        self._seg_postprocessor.configure(image_preprocessor)

    def process(self, segmentation):
        return self._seg_postprocessor.process(segmentation)


class OVDetectionPostProcessor(IDetectionPostProcessor):
    def __init__(self,
                 image_preprocessor: IImagePreProcessor,
                 output_img_size: int,
                 conf_thresh: float,
                 nms_thresh: float):
        self._image_preprocessor: IImagePreProcessor = image_preprocessor
        self._conf_thresh: float = conf_thresh
        self._nms_thresh: float = nms_thresh

        self._output_img_size: int = output_img_size

        self._max_dim: int = 0
        self._padding_top: int = 0
        self._padding_bottom: int = 0
        self._padding_left: int = 0
        self._padding_right: int = 0

        self._nms_max_number_of_boxes = 30000  # maximum number of boxes input into nms
        self._nms_max_number_of_detections = 30  # maximum number of detections per image
        self._nms_max_width_height_in_pixels = 4096

    def configure(self,
                  image_preprocessor: IImagePreProcessor,
                  output_img_size: int,
                  conf_thresh: float,
                  nms_thresh: float) -> None:
        self._image_preprocessor = image_preprocessor
        self._output_img_size = output_img_size
        self._conf_thresh = conf_thresh
        self._nms_thresh = nms_thresh

    def process(self, detections):
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

    def _perform_nms(self, detections):
        # shape prediction: (#boxes, >= 6)
        # (>= 6)er meaning: (x, y, w, h, obj_conf, cond_class_conf_1, cond_class_conf_2, ...)

        detections = self._preprocess_detections_for_nms(detections)

        boxes, scores = self._get_boxes_and_scores_for_nms(detections)
        indices = cv2.dnn.NMSBoxes(boxes, scores, 0, self._nms_thresh, top_k=self._nms_max_number_of_detections)

        output = self._postprocess_nms_output(detections, indices)

        return output

    def _preprocess_detections_for_nms(self, detections):
        detections = self._filter_by_objectness_confidence(detections)
        detections = self._calculate_class_confidence_scores(detections)
        detections = self._pin_down_last_dimension_to_6(detections)
        #detections = self._filter_by_class_confidence(detections)
        if self._too_many_boxes_remain_in(detections):
            detections = self._keep_only_boxes_with_highest_objectness_confidence(detections)

        return detections

    def _filter_by_objectness_confidence(self, detections):
        return detections[detections[:, 4] > self._conf_thresh]

    def _calculate_class_confidence_scores(self, detections):
        # class_confidence_score = conditional_class_probability * box_confidence_score (objectness)
        # p(class, object)       = p(class | object)             * p(object)
        detections[:, 5:] *= detections[:, 4:5]

        return detections

    def _pin_down_last_dimension_to_6(self, detections):
        box_coordinates = detections[:, :4]
        class_confidence_scores = detections[:, 5:]

        if self._is_multilabel_prediction(class_confidence_scores):
            i, j = (class_confidence_scores > self._conf_thresh).nonzero()
            x = np.concatenate((box_coordinates[i], class_confidence_scores[i, j, None], j[:, None]), axis=1)
        else:  # best class only
            conf = np.max(class_confidence_scores, axis=1)
            j = np.argmax(class_confidence_scores, axis=1)
            x = np.concatenate((box_coordinates, conf[:, None], j[:, None]), axis=1)[conf > self._conf_thresh]

        return x

    @staticmethod
    def _is_multilabel_prediction(class_confidence_scores) -> bool:
        return class_confidence_scores.shape[1] > 1  # makes absolutely no sense

    def _filter_by_class_confidence(self, detections):
        return detections[detections[:, 5] > self._conf_thresh]

    def _too_many_boxes_remain_in(self, prediction) -> bool:
        return prediction.shape[0] > self._nms_max_number_of_boxes

    def _keep_only_boxes_with_highest_objectness_confidence(self, prediction):
        # Sort by objectness, then discard
        return prediction[prediction[:, 4].argsort(descending=True)[:self._nms_max_number_of_boxes]]

    def _postprocess_nms_output(self, detections, nms_indices):
        detections = detections[nms_indices]
        detections = self._convert_box_coordinates(detections)

        return detections

    def _get_boxes_and_scores_for_nms(self, prediction) -> Tuple:
        # Batched NMS
        c = prediction[:, 5:6] * self._nms_max_width_height_in_pixels  # classes
        # boxes (offset by class), scores
        box_coords = prediction[:, :4].copy()
        box_coords[:, :2] += c

        return box_coords, prediction[:, 4]

    @staticmethod
    def _convert_box_coordinates(x):
        """
        Transform bounding box coordinates from xywh format (centroid coordinates, width, height) to xyxy format (upper
        left coordinates, lower right coordinates)
        :param x
        :rtype
        """
        y = np.zeros_like(x[:, :4])
        y[..., 0] = x[..., 0] - x[..., 2] / 2
        y[..., 1] = x[..., 1] - x[..., 3] / 2
        y[..., 2] = x[..., 0] + x[..., 2] / 2
        y[..., 3] = x[..., 1] + x[..., 3] / 2
        x[:, :4] = y

        return x

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


class ONNXDetectionPostProcessor(IDetectionPostProcessor):
    def __init__(self,
                 image_preprocessor: IImagePreProcessor,
                 output_img_size: int,
                 conf_thresh: float,
                 nms_thresh: float):
        self._det_postprocessor: IDetectionPostProcessor = OVDetectionPostProcessor(
            image_preprocessor=image_preprocessor,
            output_img_size=output_img_size,
            conf_thresh=conf_thresh,
            nms_thresh=nms_thresh
        )

    def process(self, detections):
        return self._det_postprocessor.process(detections)

    def configure(self,
                  image_preprocessor: IImagePreProcessor,
                  output_img_size: int,
                  conf_thresh: float,
                  nms_thresh: float) -> None:
        self._det_postprocessor.configure(
            image_preprocessor=image_preprocessor,
            output_img_size=output_img_size,
            conf_thresh=conf_thresh,
            nms_thresh=nms_thresh
        )


class TVMDetectionPostProcessor(IDetectionPostProcessor):
    def __init__(self,
                 image_preprocessor: IImagePreProcessor,
                 output_img_size: int,
                 conf_thresh: float,
                 nms_thresh: float):
        self._det_postprocessor: IDetectionPostProcessor = OVDetectionPostProcessor(
            image_preprocessor=image_preprocessor,
            output_img_size=output_img_size,
            conf_thresh=conf_thresh,
            nms_thresh=nms_thresh
        )

    def process(self, detections):
        return self._det_postprocessor.process(detections)

    def configure(self,
                  image_preprocessor: IImagePreProcessor,
                  output_img_size: int,
                  conf_thresh: float,
                  nms_thresh: float) -> None:
        self._det_postprocessor.configure(
            image_preprocessor=image_preprocessor,
            output_img_size=output_img_size,
            conf_thresh=conf_thresh,
            nms_thresh=nms_thresh
        )

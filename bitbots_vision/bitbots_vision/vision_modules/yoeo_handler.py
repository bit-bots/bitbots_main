from .candidate import CandidateFinder, Candidate
from .field_boundary import IFieldDetector

from abc import ABC, abstractmethod
from collections import defaultdict
import numpy as np
import rclpy
import os
from typing import List, Union, Dict, Any, Tuple, TYPE_CHECKING, Optional
import yaml
import cv2

from .yoeo_handler_utils import OVImagePreProcessor, OVSegmentationPostProcessor, OVDetectionPostProcessor, \
    ONNXImagePreProcessor, ONNXSegmentationPostProcessor, ONNXDetectionPostProcessor, \
    TVMImagePreProcessor, TVMSegmentationPostProcessor, TVMDetectionPostProcessor

if TYPE_CHECKING:
    from .yoeo_handler_utils import IImagePreProcessor, ISegmentationPostProcessor, IDetectionPostProcessor

logger = rclpy.logging.get_logger('vision_yoeo')

try:
    from yoeo import models as torch_models, detect as torch_detect
except ImportError:
    logger.error("Unable to import pytorchyolo. This might be fine if you use another neural network type.")

try:
    from openvino.runtime import Core as openvino_runtime_Core
except ImportError:
    logger.error("Unable to import openvino. This might be fine if you use another neural network type.")

try:
    import onnxruntime
except ImportError:
    logger.error("Unable to import onnxruntime. Thigh might be fine if you use another neural network type.")

try:
    import tvm
    from tvm.contrib import graph_executor
except ImportError:
    logger.error("Unable to import tvm. Thigh might be fine if you use another neural network type.")


class IYOEOHandler(ABC):
    @abstractmethod
    def configure(self, config: Dict) -> None:
        ...

    @abstractmethod
    def get_available_detection_class_names(self) -> List[str]:
        ...

    @abstractmethod
    def get_detection_candidates_for(self, class_name: str) -> List[Candidate]:
        ...

    @abstractmethod
    def get_available_segmentation_class_names(self) -> List[str]:
        ...

    @abstractmethod
    def get_segmentation_mask_for(self, class_name: str):
        """
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        ...

    @staticmethod
    @abstractmethod
    def model_files_exist(model_directory: str) -> bool:
        ...

    @abstractmethod
    def predict(self) -> None:
        ...

    @abstractmethod
    def set_image(self, img) -> None:
        ...


class YOEOHandlerTemplate(IYOEOHandler):
    def __init__(self, config: Dict, model_directory: str):
        logger.debug(f"Entering YOEOHandlerTemplate constructor")

        self._det_candidates: Dict = defaultdict(list)
        self._det_class_names: Union[None, List[str]] = None

        self._image = None  # : Union[None, numpy.ndarray]

        self._seg_class_names: Union[None, List[str]] = None
        self._seg_masks: Dict = dict()

        self._use_caching: bool = config['caching']

        self._load_candidate_class_names(model_directory)

        logger.debug(f"Leaving YOEOHandlerTemplate constructor")

    def configure(self, config: Dict) -> None:
        self._use_caching = config['caching']

    def _load_candidate_class_names(self, model_directory: str) -> None:
        path = YOEOPathGetter.get_names_file_path(model_directory)

        with open(path, 'r', encoding="utf-8") as fp:
            class_names = yaml.load(fp, Loader=yaml.SafeLoader)

        assert "detection" in class_names.keys(), f"Missing key 'detection' in {path}"
        assert "segmentation" in class_names.keys(), f"Missing key 'segmentation' in {path}"

        self._det_class_names = class_names['detection']
        self._seg_class_names = class_names['segmentation']

    def get_available_detection_class_names(self) -> List[str]:
        return self._det_class_names

    def get_available_segmentation_class_names(self) -> List[str]:
        return self._seg_class_names

    def get_detection_candidates_for(self, class_name: str) -> List[Candidate]:
        assert class_name in self._det_class_names, \
            f"Class '{class_name}' is not available for the current YOEO model (detection)"

        self.predict()

        return self._det_candidates[class_name]

    def get_segmentation_mask_for(self, class_name: str):
        """
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        assert class_name in self._seg_class_names, \
            f"Class '{class_name}' ist not available for the current YOEO model (segmentation)"

        self.predict()

        return self._seg_masks[class_name]

    def predict(self) -> None:
        if self._prediction_has_to_be_updated():
            logger.debug(f"Computing new prediction...")
            detections, segmentation = self._compute_new_prediction_for(self._image)
            self._create_detection_candidate_lists_from(detections)
            self._create_segmentation_masks_based_on(segmentation)

    def _prediction_has_to_be_updated(self) -> bool:
        return not self._use_caching or not (self._seg_masks or self._det_candidates)

    def _create_detection_candidate_lists_from(self, detections) -> None:
        for detection in detections:
            c = Candidate.from_x1y1x2y2(*detection[0:4].astype(int), detection[4].astype(float))
            self._det_candidates[self._det_class_names[int(detection[5])]].append(c)

    def _create_segmentation_masks_based_on(self, segmentation) -> None:
        for i, class_name in enumerate(self._seg_class_names):
            seg_mask = (segmentation == i).astype(np.uint8)
            self._seg_masks[class_name] = seg_mask

    def set_image(self, img) -> None:
        self._reset()
        self._update_image(img)

    def _reset(self) -> None:
        self._det_candidates = defaultdict(list)
        self._seg_masks = dict()

    def _update_image(self, img) -> None:
        self._image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    @staticmethod
    @abstractmethod
    def model_files_exist(model_directory: str) -> bool:
        ...

    @abstractmethod
    def _compute_new_prediction_for(self, image) -> Tuple:
        """
        """
        ...


class YOEOHandlerONNX(YOEOHandlerTemplate):
    def __init__(self, config: Dict, model_directory: str):
        logger.debug(f"Entering {self.__class__.__name__} constructor")

        super().__init__(config, model_directory)
        onnx_path = YOEOPathGetter.get_onnx_onnx_file_path(model_directory)

        logger.debug(f"Loading file...\n\t{onnx_path}")
        self._inference_session = onnxruntime.InferenceSession(onnx_path)
        self._input_layer = self._inference_session.get_inputs()[0]

        self._img_preprocessor: IImagePreProcessor = ONNXImagePreProcessor(tuple(self._input_layer.shape[2:]))
        self._det_postprocessor: IDetectionPostProcessor = ONNXDetectionPostProcessor(
            image_preprocessor=self._img_preprocessor,
            output_img_size=self._input_layer.shape[2],
            conf_thresh=config["yoeo_conf_threshold"],
            nms_thresh=config["yoeo_nms_threshold"]
        )
        self._seg_postprocessor: ISegmentationPostProcessor = ONNXSegmentationPostProcessor(self._img_preprocessor)

        logger.debug(f"Leaving {self.__class__.__name__} constructor")

    def configure(self, config: Dict) -> None:
        super().configure(config)
        self._det_postprocessor.configure(
            image_preprocessor=self._img_preprocessor,
            output_img_size=self._input_layer.shape[2],
            conf_thresh=config["yoeo_conf_threshold"],
            nms_thresh=config["yoeo_nms_threshold"]
        )

    @staticmethod
    def model_files_exist(model_directory: str) -> bool:
        return os.path.exists(YOEOPathGetter.get_onnx_onnx_file_path(model_directory))

    def _compute_new_prediction_for(self, image):
        preproccessed_image = self._img_preprocessor.process(image)

        network_input = preproccessed_image.reshape(self._input_layer.shape)
        outputs = self._inference_session.run(None, {"InputLayer": network_input.astype(np.float32)})

        detections = self._det_postprocessor.process(outputs[0])
        segmentation = self._seg_postprocessor.process(outputs[1])

        return detections, segmentation


class YOEOHandlerOpenVino(YOEOHandlerTemplate):
    def __init__(self, config: Dict, model_directory: str):
        logger.debug(f"Entering {self.__class__.__name__} constructor")

        super().__init__(config, model_directory)
        xml_path = YOEOPathGetter.get_openvino_xml_file_path(model_directory)
        bin_path = YOEOPathGetter.get_openvino_bin_file_path(model_directory)

        # https://docs.openvino.ai/latest/notebooks/002-openvino-api-with-output.html (April 9, 2022)
        self._inference_engine = openvino_runtime_Core()

        logger.debug(f"Loading files...\n\t{xml_path}\n\t{bin_path}")
        model = self._inference_engine.read_model(model=xml_path, weights=bin_path)
        device = self._select_device()

        logger.debug(f"Compiling network on device '{device}'...")
        self._compiled_model = self._inference_engine.compile_model(model=model, device_name=device)

        self._input_layer = self._compiled_model.inputs[0]
        self._output_layer_detections = self._compiled_model.outputs[0]
        self._output_layer_segmentations = self._compiled_model.outputs[1]

        _, _, height, width = self._input_layer.shape  # TODO openvino.pyopenvino.Shape...
        self._img_preprocessor: IImagePreProcessor = OVImagePreProcessor((height, width))
        self._det_postprocessor: IDetectionPostProcessor = OVDetectionPostProcessor(
            image_preprocessor=self._img_preprocessor,
            output_img_size=self._input_layer.shape[2],
            conf_thresh=config["yoeo_conf_threshold"],
            nms_thresh=config["yoeo_nms_threshold"]
        )
        self._seg_postprocessor: ISegmentationPostProcessor = OVSegmentationPostProcessor(self._img_preprocessor)

        logger.debug(f"Leaving {self.__class__.__name__} constructor")

    def _select_device(self) -> str:
        if "MYRIAD" in self._inference_engine.available_devices:  # NCS2 stick
            device = "MYRIAD"
        else:
            device = "CPU"
        return device

    def configure(self, config: Dict) -> None:
        super().configure(config)
        self._det_postprocessor.configure(
            image_preprocessor=self._img_preprocessor,
            output_img_size=self._input_layer.shape[2],
            conf_thresh=config["yoeo_conf_threshold"],
            nms_thresh=config["yoeo_nms_threshold"]
        )

    @staticmethod
    def model_files_exist(model_directory: str) -> bool:
        return os.path.exists(YOEOPathGetter.get_openvino_bin_file_path(model_directory)) and \
               os.path.exists(YOEOPathGetter.get_openvino_xml_file_path(model_directory))

    def _compute_new_prediction_for(self, image):
        preproccessed_image = self._img_preprocessor.process(image)

        network_input = preproccessed_image.reshape(self._input_layer.shape)
        network_output = self._compiled_model(inputs=[network_input])

        detections = self._det_postprocessor.process(network_output[self._output_layer_detections])
        segmentation = self._seg_postprocessor.process(network_output[self._output_layer_segmentations])

        return detections, segmentation


class YOEOHandlerPytorch(YOEOHandlerTemplate):
    """
    Using Pytorch to get YOEO predictions
    """

    def __init__(self, config, model_directory):
        logger.debug(f"Entering {self.__class__.__name__} constructor")

        super().__init__(config, model_directory)

        config_path = YOEOPathGetter.get_pytorch_cfg_file(model_directory)
        weights_path = YOEOPathGetter.get_pytorch_pth_file(model_directory)

        logger.debug(f"Loading files...\n\t{config_path}\n\t{weights_path}")
        self._model = torch_models.load_model(config_path, weights_path)

        self._conf_thresh: float = config["yoeo_conf_threshold"]
        self._nms_thresh: float = config["yoeo_nms_threshold"]

        logger.debug(f"Leaving {self.__class__.__name__} constructor")

    def configure(self, config: Dict) -> None:
        super().configure(config)
        self._conf_thresh = config["yoeo_conf_threshold"]
        self._nms_thresh = config["yoeo_nms_threshold"]

    @staticmethod
    def model_files_exist(model_directory: str) -> bool:
        return os.path.exists(YOEOPathGetter.get_pytorch_cfg_file(model_directory)) and \
               os.path.exists(YOEOPathGetter.get_pytorch_pth_file(model_directory))

    def _compute_new_prediction_for(self, image):
        detections, segmentation = torch_detect.detect_image(
            self._model,
            image,
            conf_thres=self._conf_thresh,
            nms_thres=self._nms_thresh
        )

        segmentation = self._postprocess_segmentation(segmentation)

        return detections, segmentation

    @staticmethod
    def _postprocess_segmentation(segmentations: Any):
        return np.moveaxis(segmentations, 0, -1)


class YOEOHandlerTVM(YOEOHandlerTemplate):
    def __init__(self, config: Dict, model_directory: str):
        logger.debug(f"Entering {self.__class__.__name__} constructor")

        super().__init__(config, model_directory)

        json_path = YOEOPathGetter.get_tvm_json_file_path(model_directory)
        params_path = YOEOPathGetter.get_tvm_params_file_path(model_directory)
        binary_path = YOEOPathGetter.get_tvm_so_file_path(model_directory)

        logger.debug(f"Loading files...\n\t{binary_path}\n\t{params_path}\n\t{json_path}")
        binary_lib = tvm.runtime.load_module(binary_path)
        loaded_params = bytearray(open(params_path, "rb").read())
        loaded_json = open(json_path).read()

        device = self._select_device()

        logger.debug(f"Creating network on device '{device}'...")
        self._model = graph_executor.create(loaded_json, binary_lib, device)
        self._model.load_params(loaded_params)

        input_shape_dict, _ = self._model.get_input_info()
        self._input_layer_shape = input_shape_dict.get('InputLayer')

        height, width = self._input_layer_shape[2], self._input_layer_shape[3]
        self._img_preprocessor: IImagePreProcessor = TVMImagePreProcessor((height, width))
        self._det_postprocessor: IDetectionPostProcessor = TVMDetectionPostProcessor(
            image_preprocessor=self._img_preprocessor,
            output_img_size=self._input_layer_shape[2],
            conf_thresh=config["yoeo_conf_threshold"],
            nms_thresh=config["yoeo_nms_threshold"]
        )
        self._seg_postprocessor: ISegmentationPostProcessor = TVMSegmentationPostProcessor(self._img_preprocessor)

        logger.debug(f"Leaving {self.__class__.__name__} constructor")

    @staticmethod
    def _select_device() -> tvm.runtime.Device:
        if tvm.vulkan().exist:
            return tvm.vulkan()
        else:
            return tvm.cpu()

    def configure(self, config: Dict) -> None:
        super().configure(config)
        self._det_postprocessor.configure(
            image_preprocessor=self._img_preprocessor,
            output_img_size=self._input_layer_shape[2],
            conf_thresh=config["yoeo_conf_threshold"],
            nms_thresh=config["yoeo_nms_threshold"]
        )

    @staticmethod
    def model_files_exist(model_directory: str) -> bool:
        return os.path.exists(YOEOPathGetter.get_tvm_json_file_path(model_directory)) and \
               os.path.exists(YOEOPathGetter.get_tvm_params_file_path(model_directory)) and \
               os.path.exists(YOEOPathGetter.get_tvm_so_file_path(model_directory))

    def _compute_new_prediction_for(self, image):
        preproccessed_image = self._img_preprocessor.process(image)
        network_input = preproccessed_image.reshape(self._input_layer_shape)

        self._model.set_input(InputLayer=network_input)
        self._model.run()

        detections = self._det_postprocessor.process(self._model.get_output(0).numpy())
        segmentation = self._seg_postprocessor.process(self._model.get_output(1).numpy())

        return detections, segmentation


class YOEOPathGetter:
    @classmethod
    def _assemble_full_path(cls, model_directory: str, subdir: Optional[str], filename: str) -> str:
        if subdir is None:
            path = os.path.join(model_directory, filename)
        else:
            path = os.path.join(model_directory, subdir, filename)

        return path

    @classmethod
    def get_names_file_path(cls, model_directory) -> str:
        return cls._assemble_full_path(model_directory, None, "yoeo_names.yaml")

    @classmethod
    def get_onnx_onnx_file_path(cls, model_directory) -> str:
        return cls._assemble_full_path(model_directory, "onnx", "yoeo.onnx")

    @classmethod
    def get_openvino_bin_file_path(cls, model_directory: str) -> str:
        return cls._assemble_full_path(model_directory, "openvino", "yoeo.bin")

    @classmethod
    def get_openvino_xml_file_path(cls, model_directory) -> str:
        return cls._assemble_full_path(model_directory, "openvino", "yoeo.xml")

    @classmethod
    def get_pytorch_cfg_file(cls, model_directory: str) -> str:
        return cls._assemble_full_path(model_directory, "pytorch", "yoeo.cfg")

    @classmethod
    def get_pytorch_pth_file(cls, model_directory: str) -> str:
        return cls._assemble_full_path(model_directory, "pytorch", "yoeo.pth")

    @classmethod
    def get_tvm_json_file_path(cls, model_directory: str) -> str:
        return cls._assemble_full_path(model_directory, "tvm", "yoeo.json")

    @classmethod
    def get_tvm_params_file_path(cls, model_directory: str) -> str:
        return cls._assemble_full_path(model_directory, "tvm", "yoeo.params")

    @classmethod
    def get_tvm_so_file_path(cls, model_directory: str) -> str:
        return cls._assemble_full_path(model_directory, "tvm", "yoeo.so")


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

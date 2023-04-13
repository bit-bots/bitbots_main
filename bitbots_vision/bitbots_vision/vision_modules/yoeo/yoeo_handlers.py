from __future__ import annotations

import cv2
import numpy as np
import os
import rclpy
import yaml
from abc import ABC, abstractmethod
from collections import defaultdict
from typing import List, Dict, Tuple, Optional

from . import utils
from bitbots_vision.vision_modules.candidate import Candidate


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
    """
    Interface of the YOEO handlers.
    """

    @abstractmethod
    def configure(self, config: Dict) -> None:
        """
        Allows to (re-) configure the YOEO handler.
        """
        ...

    @abstractmethod
    def get_available_detection_class_names(self) -> List[str]:
        """
        Returns the names of the classes that are part of the YOEO detection.
        """
        ...

    @abstractmethod
    def get_detection_candidates_for(self, class_name: str) -> List[Candidate]:
        """
        Returns the detection candidates for a given class.
        """
        ...

    @abstractmethod
    def get_available_segmentation_class_names(self) -> List[str]:
        """
        Returns the names of the classes that are part of the YOEO segmentation.
        """
        ...

    @abstractmethod
    def get_segmentation_mask_for(self, class_name: str) -> np.ndarray:
        """
        Returns the segmentation mask for a given class.

        :return: the segmentation mask for class_name
        :rtype: numpy.ndarray(shape=(height, width, 1))
        """
        ...

    @staticmethod
    @abstractmethod
    def model_files_exist(model_directory: str) -> bool:
        """
        Checks whether the necessary model files exists in the model directory.

        :return: true if files exist, false if at least one file ist missing
        :rtype: bool
        """
        ...

    @abstractmethod
    def predict(self) -> None:
        """
        Runs the YOEO network (if necessary) on the current input image.
        """
        ...

    @abstractmethod
    def set_image(self, image: np.ndarray) -> None:
        """
        A subsequent call to predict() will run the YOEO network on this image.

        :param image: the image to run the YOEO network on
        :type image: np.ndarray
        """
        ...


class YOEOHandlerTemplate(IYOEOHandler):
    """
    Abstract base implementation of the IYOEOHandler interface. Actual YOEO handlers need to only implement the
    following two hook methods if they inherit from this template:
        - model_files_exist(model_directory: str) -> bool:
        - _compute_new_prediction_for(self, image) -> Tuple:
    """
    def __init__(self, config: Dict, model_directory: str):
        logger.debug(f"Entering YOEOHandlerTemplate constructor")

        self._det_candidates: Dict = defaultdict(list)
        self._det_class_names: Optional[List[str]] = None

        self._image: Optional[np.ndarray] = None

        self._prediction_is_up_to_date: bool = True

        self._seg_class_names: Optional[List[str]] = None
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

    def get_robot_class_ids(self) -> List[int]:
        ids = []
        for i, c in enumerate(self._det_class_names):
            if "robot" in c:
                ids.append(i)
        return ids

    def get_segmentation_mask_for(self, class_name: str):
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

            self._prediction_is_up_to_date = True

    def _prediction_has_to_be_updated(self) -> bool:
        return not self._use_caching or not self._prediction_is_up_to_date

    def _create_detection_candidate_lists_from(self, detections: np.ndarray) -> None:
        for detection in detections:
            c = Candidate.from_x1y1x2y2(*detection[0:4].astype(int), detection[4].astype(float))
            self._det_candidates[self._det_class_names[int(detection[5])]].append(c)

    def _create_segmentation_masks_based_on(self, segmentation) -> None:
        for i, class_name in enumerate(self._seg_class_names):
            seg_mask = (segmentation == i).astype(np.uint8)
            self._seg_masks[class_name] = seg_mask

    def set_image(self, image: np.ndarray) -> None:
        self._reset()
        self._update_image(image)

    def _reset(self) -> None:
        self._det_candidates = defaultdict(list)
        self._seg_masks = dict()

    def _update_image(self, img: np.ndarray) -> None:
        self._image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self._prediction_is_up_to_date = False

    @staticmethod
    @abstractmethod
    def model_files_exist(model_directory: str) -> bool:
        ...

    @abstractmethod
    def _compute_new_prediction_for(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Hook method to be implemented by actual YOEO handlers.

        :param image: the image that should be input into the network
        :type image: np.ndarray
        :return: post-processed YOEO detections and segmentations (in this order)
        :rtype: Tuple[np.ndarray, np.ndarray]
        """
        ...


class YOEOHandlerONNX(YOEOHandlerTemplate):
    """
    YOEO handler for the ONNX framework.

    Framework version: 1.12.0
    see https://onnxruntime.ai/docs/get-started/with-python.html for ONNX documentation
    """
    def __init__(self, config: Dict, model_directory: str):
        super().__init__(config, model_directory)

        logger.debug(f"Entering {self.__class__.__name__} constructor")

        onnx_path = YOEOPathGetter.get_onnx_onnx_file_path(model_directory)

        logger.debug(f"Loading file...\n\t{onnx_path}")
        self._inference_session = onnxruntime.InferenceSession(onnx_path)
        self._input_layer = self._inference_session.get_inputs()[0]

        self._img_preprocessor: utils.IImagePreProcessor = utils.DefaultImagePreProcessor(
            tuple(self._input_layer.shape[2:])
        )
        self._det_postprocessor: utils.IDetectionPostProcessor = utils.DefaultDetectionPostProcessor(
            image_preprocessor=self._img_preprocessor,
            output_img_size=self._input_layer.shape[2],
            conf_thresh=config["yoeo_conf_threshold"],
            nms_thresh=config["yoeo_nms_threshold"],
            robot_class_ids=self.get_robot_class_ids()
        )
        self._seg_postprocessor: utils.ISegmentationPostProcessor = utils.DefaultSegmentationPostProcessor(
            self._img_preprocessor
        )

        logger.debug(f"Leaving {self.__class__.__name__} constructor")

    def configure(self, config: Dict) -> None:
        super().configure(config)
        self._det_postprocessor.configure(image_preprocessor=self._img_preprocessor,
                                          output_img_size=self._input_layer.shape[2],
                                          conf_thresh=config["yoeo_conf_threshold"],
                                          nms_thresh=config["yoeo_nms_threshold"],
                                          robot_class_ids=self.get_robot_class_ids())

    @staticmethod
    def model_files_exist(model_directory: str) -> bool:
        return os.path.exists(YOEOPathGetter.get_onnx_onnx_file_path(model_directory))

    def _compute_new_prediction_for(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        preproccessed_image = self._img_preprocessor.process(image)

        network_input = preproccessed_image.reshape(self._input_layer.shape)
        outputs = self._inference_session.run(None, {"InputLayer": network_input.astype(np.float32)})

        detections = self._det_postprocessor.process(outputs[0])
        segmentation = self._seg_postprocessor.process(outputs[1])

        return detections, segmentation


class YOEOHandlerOpenVino(YOEOHandlerTemplate):
    """
    YOEO handler for the OpenVino framework.

    Framework version: OpenVINO 2022.1
    Code is based on https://docs.openvino.ai/latest/notebooks/002-openvino-api-with-output.html (April 9, 2022)
    """
    def __init__(self, config: Dict, model_directory: str):
        super().__init__(config, model_directory)

        logger.debug(f"Entering {self.__class__.__name__} constructor")

        xml_path = YOEOPathGetter.get_openvino_xml_file_path(model_directory)
        bin_path = YOEOPathGetter.get_openvino_bin_file_path(model_directory)

        self._inference_engine = openvino_runtime_Core()

        logger.debug(f"Loading files...\n\t{xml_path}\n\t{bin_path}")
        model = self._inference_engine.read_model(model=xml_path, weights=bin_path)
        device = self._select_device()

        logger.debug(f"Compiling network on device '{device}'...")
        self._compiled_model = self._inference_engine.compile_model(model=model, device_name=device)

        self._input_layer = self._compiled_model.inputs[0]
        self._output_layer_detections = self._compiled_model.outputs[0]
        self._output_layer_segmentations = self._compiled_model.outputs[1]

        _, _, height, width = self._input_layer.shape
        self._img_preprocessor: utils.IImagePreProcessor = utils.DefaultImagePreProcessor((height, width))
        self._det_postprocessor: utils.IDetectionPostProcessor = utils.DefaultDetectionPostProcessor(
            image_preprocessor=self._img_preprocessor,
            output_img_size=self._input_layer.shape[2],
            conf_thresh=config["yoeo_conf_threshold"],
            nms_thresh=config["yoeo_nms_threshold"],
            robot_class_ids=self.get_robot_class_ids()
        )
        self._seg_postprocessor: utils.ISegmentationPostProcessor = utils.DefaultSegmentationPostProcessor(
            self._img_preprocessor
        )

        logger.debug(f"Leaving {self.__class__.__name__} constructor")

    def _select_device(self) -> str:
        if "MYRIAD" in self._inference_engine.available_devices:  # NCS2 stick
            device = "MYRIAD"
        else:
            device = "CPU"
        return device

    def configure(self, config: Dict) -> None:
        super().configure(config)
        self._det_postprocessor.configure(image_preprocessor=self._img_preprocessor,
                                          output_img_size=self._input_layer.shape[2],
                                          conf_thresh=config["yoeo_conf_threshold"],
                                          nms_thresh=config["yoeo_nms_threshold"],
                                          robot_class_ids=self.get_robot_class_ids())

    @staticmethod
    def model_files_exist(model_directory: str) -> bool:
        return os.path.exists(YOEOPathGetter.get_openvino_bin_file_path(model_directory)) and \
               os.path.exists(YOEOPathGetter.get_openvino_xml_file_path(model_directory))

    def _compute_new_prediction_for(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        preproccessed_image = self._img_preprocessor.process(image)

        network_input = preproccessed_image.reshape(self._input_layer.shape)
        network_output = self._compiled_model(inputs=[network_input])

        detections = self._det_postprocessor.process(network_output[self._output_layer_detections])
        segmentation = self._seg_postprocessor.process(network_output[self._output_layer_segmentations])

        return detections, segmentation


class YOEOHandlerPytorch(YOEOHandlerTemplate):
    """
    YOEO handler for the PyTorch framework
    """
    def __init__(self, config, model_directory):
        super().__init__(config, model_directory)

        logger.debug(f"Entering {self.__class__.__name__} constructor")

        config_path = YOEOPathGetter.get_pytorch_cfg_file_path(model_directory)
        weights_path = YOEOPathGetter.get_pytorch_pth_file_path(model_directory)

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
        return os.path.exists(YOEOPathGetter.get_pytorch_cfg_file_path(model_directory)) and \
               os.path.exists(YOEOPathGetter.get_pytorch_pth_file_path(model_directory))

    def _compute_new_prediction_for(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        detections, segmentation = torch_detect.detect_image(self._model,
                                                             image,
                                                             conf_thres=self._conf_thresh,
                                                             nms_thres=self._nms_thresh)

        segmentation = self._postprocess_segmentation(segmentation)

        return detections, segmentation

    @staticmethod
    def _postprocess_segmentation(segmentations: np.ndarray) -> np.ndarray:
        return np.moveaxis(segmentations, 0, -1)


class YOEOHandlerTVM(YOEOHandlerTemplate):
    """
    YOEO handler for the TVM framework.
    """
    def __init__(self, config: Dict, model_directory: str):
        super().__init__(config, model_directory)

        logger.debug(f"Entering {self.__class__.__name__} constructor")

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
        self._img_preprocessor: utils.IImagePreProcessor = utils.DefaultImagePreProcessor((height, width))
        self._det_postprocessor: utils.IDetectionPostProcessor = utils.DefaultDetectionPostProcessor(
            image_preprocessor=self._img_preprocessor,
            output_img_size=self._input_layer_shape[2],
            conf_thresh=config["yoeo_conf_threshold"],
            nms_thresh=config["yoeo_nms_threshold"],
            robot_class_ids=self.get_robot_class_ids()
        )
        self._seg_postprocessor: utils.ISegmentationPostProcessor = utils.DefaultSegmentationPostProcessor(
            self._img_preprocessor
        )

        logger.debug(f"Leaving {self.__class__.__name__} constructor")

    @staticmethod
    def _select_device() -> tvm.runtime.Device:
        if tvm.vulkan().exist:
            return tvm.vulkan()
        else:
            return tvm.cpu()

    def configure(self, config: Dict) -> None:
        super().configure(config)
        self._det_postprocessor.configure(image_preprocessor=self._img_preprocessor,
                                          output_img_size=self._input_layer_shape[2],
                                          conf_thresh=config["yoeo_conf_threshold"],
                                          nms_thresh=config["yoeo_nms_threshold"],
                                          robot_class_ids=self.get_robot_class_ids())

    @staticmethod
    def model_files_exist(model_directory: str) -> bool:
        return os.path.exists(YOEOPathGetter.get_tvm_json_file_path(model_directory)) and \
               os.path.exists(YOEOPathGetter.get_tvm_params_file_path(model_directory)) and \
               os.path.exists(YOEOPathGetter.get_tvm_so_file_path(model_directory))

    def _compute_new_prediction_for(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        preproccessed_image = self._img_preprocessor.process(image)
        network_input = preproccessed_image.reshape(self._input_layer_shape)

        self._model.set_input(InputLayer=network_input)
        self._model.run()

        detections = self._det_postprocessor.process(self._model.get_output(0).numpy())
        segmentation = self._seg_postprocessor.process(self._model.get_output(1).numpy())

        return detections, segmentation


class YOEOPathGetter:
    """
    PathGetter class for all YOEO handlers. They idea behind this class is to have all path information in one place so
    that it is easier to change this information if changes to the directory structure should become necessary.
    Depending on the actual YOEO handler, certain framework specific methods provide the paths to the necessary files.
    To make it more clear which methods to use, the framework specific methods follow the following naming scheme:
    "get_'FRAMEWORK-NAME'_'FILE-EXTENSION'_file_path"
    """
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
    def get_pytorch_cfg_file_path(cls, model_directory: str) -> str:
        return cls._assemble_full_path(model_directory, "pytorch", "yoeo.cfg")

    @classmethod
    def get_pytorch_pth_file_path(cls, model_directory: str) -> str:
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

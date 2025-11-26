import os
from typing import Optional

import rclpy

from bitbots_vision.vision_modules import ros_utils

from . import yoeo_handlers
from .model_config import ModelConfig, ModelConfigLoader

logger = rclpy.logging.get_logger("bitbots_vision")


class YOEOObjectManager:
    """
    This class manages the creation and update of the YOEO handler instance.
    """

    _HANDLERS_BY_NAME: dict[str, type[yoeo_handlers.YOEOHandlerTemplate]] = {
        "openvino": yoeo_handlers.YOEOHandlerOpenVino,
        "onnx": yoeo_handlers.YOEOHandlerONNX,
        "pytorch": yoeo_handlers.YOEOHandlerPytorch,
        "tvm": yoeo_handlers.YOEOHandlerTVM,
    }

    _config: dict = {}
    _framework: str = ""
    _model_config: ModelConfig = ModelConfig()
    _model_path: str = ""
    _yoeo_instance: Optional[yoeo_handlers.IYOEOHandler] = None

    @classmethod
    def get(cls) -> yoeo_handlers.IYOEOHandler:
        """
        Get the current YOEO handler instance.

        :return: the current YOEO handler instance
        :rtype: IYOEOHandler
        """
        assert cls._yoeo_instance is not None, "YOEO handler instance not set!"
        return cls._yoeo_instance

    @classmethod
    def get_id(cls) -> int:
        """
        Get the (Python) ID of the current YOEO handler instance.

        :return: the ID of the current YOEO handler instance
        :rtype: int
        """
        assert cls._yoeo_instance is not None, "YOEO handler instance not set!"
        return id(cls._yoeo_instance)

    @classmethod
    def is_team_color_detection_supported(cls) -> bool:
        """
        Whether the current YOEO object provides team color detection.

        :return: true if team color detection is provided, false otherwise
        :rtype: bool
        """
        return cls._model_config.team_colors_are_provided()

    @classmethod
    def configure(cls, config: dict) -> None:
        framework = config["yoeo_framework"]
        cls._verify_framework_parameter(framework)

        conda_prefix = os.environ.get("CONDA_PREFIX", "")
        if not conda_prefix:
            raise ValueError("CONDA_PREFIX environment variable not set! We now expect YOEO models to be shared as conda packages.")

        # Assemble model package name and look at its share directory
        model_path = os.path.join(
            conda_prefix,
            "share",
            "bitbots_model_" + config["yoeo_model_path"])

        cls._verify_required_neural_network_files_exist(framework, model_path)

        cls._configure_yoeo_instance(config, framework, model_path)

        cls._config = config
        cls._framework = framework
        cls._model_path = model_path

    @staticmethod
    def _verify_framework_parameter(framework: str) -> None:
        if framework not in {"openvino", "onnx", "pytorch", "tvm"}:
            logger.error(f"Unknown neural network framework '{framework}'")

    @classmethod
    def _verify_required_neural_network_files_exist(cls, framework: str, model_path: str) -> None:
        if not cls._model_files_exist(framework, model_path):
            logger.error("No matching model file(s) found!")

    @classmethod
    def _model_files_exist(cls, framework: str, model_path: str) -> bool:
        return cls._HANDLERS_BY_NAME[framework].model_files_exist(model_path)

    @classmethod
    def _configure_yoeo_instance(cls, config: dict, framework: str, model_path: str) -> None:
        if cls._new_yoeo_handler_is_needed(framework, model_path):
            cls._load_model_config(model_path)
            cls._instantiate_new_yoeo_handler(config, framework, model_path)
        elif cls._yoeo_parameters_have_changed(config):
            assert cls._yoeo_instance is not None, "YOEO handler instance not set!"
            cls._yoeo_instance.configure(config)

    @classmethod
    def _new_yoeo_handler_is_needed(cls, framework: str, model_path: str) -> bool:
        return cls._yoeo_instance is None or cls._framework != framework or cls._model_path != model_path

    @classmethod
    def _load_model_config(cls, model_path: str) -> None:
        cls._model_config = ModelConfigLoader.load_from(model_path)

    @classmethod
    def _instantiate_new_yoeo_handler(cls, config: dict, framework: str, model_path: str) -> None:
        cls._yoeo_instance = cls._HANDLERS_BY_NAME[framework](
            config,
            model_path,
            cls._model_config.get_detection_classes(),
            cls._model_config.get_robot_class_ids(),
            cls._model_config.get_segmentation_classes(),
        )
        logger.info(f"Using {cls._yoeo_instance.__class__.__name__}")

    @classmethod
    def _yoeo_parameters_have_changed(cls, new_config: dict) -> bool:
        return ros_utils.config_param_change(cls._config, new_config, r"yoeo_")

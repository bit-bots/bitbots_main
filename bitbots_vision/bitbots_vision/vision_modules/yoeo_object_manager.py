import os.path as osp
import rclpy
from typing import Optional, Dict


from . import yoeo_handler, ros_utils


logger = rclpy.logging.get_logger('bitbots_vision')


class ObjectManager:
    _config: Dict = {}
    _framework: str = ""
    _package_directory: str = ""
    _package_directory_set: bool = False
    _yoeo_instance: Optional[yoeo_handler.IYOEOHandler] = None

    @classmethod
    def set_package_directory(cls, package_directory: str) -> None:
        cls._package_directory = package_directory
        cls._package_directory_set = True

    @classmethod
    def get(cls) -> yoeo_handler.IYOEOHandler:
        if cls._yoeo_instance is None:
            logger.error("No yoeo handler created yet!")
        else:
            return cls._yoeo_instance

    @classmethod
    def configure(cls, config: Dict) -> None:
        if not cls._package_directory_set:
            logger.error("Package directory not set!")

        framework = config["yoeo_framework"]
        cls._verify_framework_parameter(framework)

        model_path = cls._get_full_model_path(config["yoeo_model_path"])
        cls._verify_required_neural_network_files_exist(framework, model_path)

        cls._configure_yoeo_instance(config, framework, model_path)

        cls._config = config
        cls._framework = framework

    @staticmethod
    def _verify_framework_parameter(framework: str) -> None:
        if framework not in {'openvino', 'onnx', 'pytorch', 'tvm'}:
            logger.error(f"Unknown neural network framework '{framework}'")

    @classmethod
    def _get_full_model_path(cls, model_path: str) -> str:
        return osp.join(cls._package_directory, 'models', model_path)

    @classmethod
    def _verify_required_neural_network_files_exist(cls, framework: str, model_path: str) -> None:
        if not cls._model_files_exist(framework, model_path):
            logger.error("No matching model file(s) found!")

    @staticmethod
    def _model_files_exist(framework: str, model_path: str) -> bool:
        exists: bool = False

        if framework == "openvino":
            exists = yoeo_handler.YOEOHandlerOpenVino.model_files_exist(model_path)
        elif framework == "onnx":
            exists = yoeo_handler.YOEOHandlerONNX.model_files_exist(model_path)
        elif framework == "pytorch":
            exists = yoeo_handler.YOEOHandlerPytorch.model_files_exist(model_path)
        elif framework == "tvm":
            exists = yoeo_handler.YOEOHandlerTVM.model_files_exist(model_path)

        return exists

    @classmethod
    def _configure_yoeo_instance(cls, config: Dict, framework: str, model_path: str) -> None:
        logger.error(model_path)
        if cls._new_yoeo_handler_is_needed(framework):
            cls._instantiate_new_yoeo_handler(config, framework, model_path)
        elif cls._yoeo_parameters_have_changed(config):
            cls._yoeo_instance.configure(config)

    @classmethod
    def _new_yoeo_handler_is_needed(cls, framework: str) -> bool:
        return cls._yoeo_instance is None or cls._framework != framework

    @classmethod
    def _instantiate_new_yoeo_handler(cls, config: Dict, framework: str, model_path: str) -> None:
        if framework == "openvino":
            cls._yoeo_instance = yoeo_handler.YOEOHandlerOpenVino(config, model_path)
        elif framework == "onnx":
            cls._yoeo_instance = yoeo_handler.YOEOHandlerONNX(config, model_path)
        elif framework == "pytorch":
            cls._yoeo_instance = yoeo_handler.YOEOHandlerPytorch(config, model_path)
        elif framework == "tvm":
            cls._yoeo_instance = yoeo_handler.YOEOHandlerTVM(config, model_path)
        logger.info(f"Using {cls._yoeo_instance.__class__.__name__}")

    @classmethod
    def _yoeo_parameters_have_changed(cls, new_config: Dict) -> bool:
        return ros_utils.config_param_change(cls._config, new_config, r'yoeo_')

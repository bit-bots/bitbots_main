#! /usr/bin/env python3
from typing import Dict, Union, List

import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from copy import deepcopy
from cv_bridge import CvBridge
from threading import Lock
from sensor_msgs.msg import Image
from bitbots_vision.vision_modules import yoeo_handler, ros_utils
from bitbots_vision.vision_modules.yoeo_vision_components import IVisionComponent, CameraCapCheckComponent, \
    YOEOBallDetectionComponent, YOEOObstacleDetectionComponent, YOEOGoalpostDetectionComponent, \
    YOEOLineDetectionComponent, YOEOFieldBoundaryDetectionComponent, YOEOFieldDetectionComponent, DebugImageComponent

from .yoeo_params import gen

logger = rclpy.logging.get_logger('bitbots_vision')

try:
    from profilehooks import profile, \
        timecall  # Profilehooks profiles certain functions in you add the @profile or @timecall decorator.
except ImportError:
    profile = lambda x: x
    logger.info("No Profiling avalabile")


class YOEOVision(Node):
    """
    The Vision is the main ROS-node for handling all tasks related to image processing.

    This class defines the whole image processing pipeline, which uses the modules from the `vision_modules`.
    It also handles the dynamic reconfiguration of the bitbots_vision.
    """

    def __init__(self):
        super().__init__('bitbots_vision')

        self._package_path = get_package_share_directory('bitbots_vision')

        logger.info('Initializing vision...')

        self._cv_bridge = CvBridge()

        self._config: Dict = {}

        self._sub_image = None

        # Reconfigure data transfer variable
        self._transfer_reconfigure_data = None
        self._transfer_reconfigure_data_mutex = Lock()

        # Image transfer variable
        self._transfer_image_msg = None
        self._transfer_image_msg_mutex = Lock()

        self._yoeo_handler: Union[None, yoeo_handler.IYOEOHandler] = None
        self._vision_components: Union[None, List[IVisionComponent]] = None

        # Setup reconfiguration
        gen.declare_params(self)
        self.add_on_set_parameters_callback(self._dynamic_reconfigure_callback)

        # Add general params
        ros_utils.set_general_parameters(["caching"])

        self._dynamic_reconfigure_callback(self.get_parameters_by_prefix("").values())

        logger.debug("Constructor ran")

    def _dynamic_reconfigure_callback(self, params) -> SetParametersResult:
        """
        Callback for the dynamic reconfigure configuration.

        :param params: new config # TODO
        """
        logger.info(f"type params: {type(params)}")
        new_config = self._get_updated_config_with(params)
        self._configure_vision(new_config)
        self._config = new_config

        return SetParametersResult(successful=True)

    def _get_updated_config_with(self, params) -> Dict:
        new_config = deepcopy(self._config)
        for param in params:
            new_config[param.name] = param.value
        return new_config

    def _configure_vision(self, new_config: Dict) -> None:
        logger.debug("Start Configure_vision".upper())

        neural_network_type = new_config["neural_network_type"]
        model_path = self._get_model_path(new_config)

        self._verify_neural_network_type(neural_network_type)
        self._verify_neural_network_files_exist(neural_network_type, model_path)

        self._set_up_yoeo_handler(new_config)
        self._set_up_vision_components(new_config)

        self._register_subscribers(new_config)

        logger.debug("End Configure_vision".upper())

    def _get_model_path(self, config: Dict) -> str:
        return os.path.join(self._package_path, 'models', config['yoeo_model_path'])

    @staticmethod
    def _verify_neural_network_type(neural_network_type: str) -> None:
        if neural_network_type not in {'openvino', 'onnx', 'pytorch'}:
            logger.error(f"Unknown neural network type '{neural_network_type}'")

    def _verify_neural_network_files_exist(self, neural_network_type: str, model_path: str) -> None:
        if not self._model_files_exist(neural_network_type, model_path):
            logger.error("No matching model file(s) found!")

    @staticmethod
    def _model_files_exist(neural_network_type: str, model_path: str) -> bool:
        exists: bool = False
        if neural_network_type == "openvino":
            exists = os.path.exists(os.path.join(model_path, "yoeo.xml")) and \
                     os.path.exists(os.path.join(model_path, "yoeo.bin"))
        elif neural_network_type == "onnx":
            exists = os.path.exists(os.path.join(model_path, "yoeo.onnx"))
        elif neural_network_type == "pytorch":
            exists = os.path.exists(os.path.join(model_path, "yoeo.pth"))
        return exists

    def _set_up_yoeo_handler(self, new_config: Dict) -> None:
        if self._new_yoeo_handler_is_needed(new_config):
            self._instantiate_new_yoeo_handler(new_config)
        elif self._yoeo_parameters_have_changed(new_config):
            self._yoeo_handler.set_config(new_config)

    def _new_yoeo_handler_is_needed(self, new_config: Dict) -> bool:
        return self._yoeo_handler is None or \
               ros_utils.config_param_change(self._config, new_config, ['neural_network_type'])

    def _instantiate_new_yoeo_handler(self, new_config: Dict) -> None:
        neural_network_type = new_config["neural_network_type"]
        model_path = self._get_model_path(new_config)
        if neural_network_type == "openvino":
            self._yoeo_handler = yoeo_handler.YOEOHandlerOpenVino(new_config, model_path)
        elif neural_network_type == "onnx":
            self._yoeo_handler = yoeo_handler.YOEOHandlerONNX(new_config, model_path)
        elif neural_network_type == "pytorch":
            self._yoeo_handler = yoeo_handler.YOEOHandlerPytorch(new_config, model_path)
        logger.info(f"Using {self._yoeo_handler.__class__.__name__}")

    def _yoeo_parameters_have_changed(self, new_config: Dict) -> bool:
        return ros_utils.config_param_change(self._config, new_config, r'yoeo_')

    def _set_up_vision_components(self, new_config: Dict) -> None:
        self._vision_components = []

        if new_config["component_camera_cap_check_active"]:
            self._vision_components.append(CameraCapCheckComponent(self))
        if new_config["component_ball_detection_active"]:
            self._vision_components.append(YOEOBallDetectionComponent(self))
        if new_config["component_obstacle_detection_active"]:
            self._vision_components.append(YOEOObstacleDetectionComponent(self))
        if new_config["component_goalpost_detection_active"]:
            self._vision_components.append(YOEOGoalpostDetectionComponent(self))
        if new_config["component_line_detection_active"]:
            self._vision_components.append(YOEOLineDetectionComponent(self))
        if new_config["component_field_boundary_detection_active"]:
            self._vision_components.append(YOEOFieldBoundaryDetectionComponent(self))
        if new_config["component_field_detection_active"]:
            self._vision_components.append(YOEOFieldDetectionComponent(self))
        if new_config["component_debug_image_active"]:
            self._vision_components.append(DebugImageComponent(self))

        for vision_component in self._vision_components:
            vision_component.configure(new_config, self._yoeo_handler)

    def _register_subscribers(self, config: Dict) -> None:
        self._sub_image = ros_utils.create_or_update_subscriber(self, self._config, config, self._sub_image,
                                                                'ROS_img_msg_topic', Image,
                                                                callback=self._image_callback)

    def _image_callback(self, image_msg) -> None:
        # TODO: type
        """
        This method is called by the Image-message subscriber.
        Too old Image-Messages are dropped.

        Sometimes the queue gets too large, even if the size is limited to 1.
        That is why we drop old images manually.
        """
        logger.debug("_image_callback called")
        if self._image_is_too_old(image_msg) or self._transfer_image_msg_mutex_is_locked():
            return

        with self._transfer_image_msg_mutex:
            self._handle_image(image_msg)

    def _image_is_too_old(self, image_msg) -> bool:
        image_age = self.get_clock().now() - rclpy.time.Time.from_msg(image_msg.header.stamp)
        if 1.0 < image_age.nanoseconds / 1000000000 < 1000.0:
            logger.warning(
                f"Vision: Dropping incoming Image-Message because it is too old! ({image_age.to_sec()} sec)")
            return True
        else:
            return False

    def _transfer_image_msg_mutex_is_locked(self) -> bool:
        return self._transfer_image_msg_mutex.locked()

    @profile
    def _handle_image(self, image_msg):
        """
        Run the vision pipeline
        """

        image = self._convert_image_msg_to_cv2_image(image_msg)

        if image is None:
            logger.debug("Image content is None :(")
            return

        self._yoeo_handler.set_image(image)
        self._publish_image_to_components(image)

        self._yoeo_handler.predict()
        self._run_components(image_msg)

    def _convert_image_msg_to_cv2_image(self, image_msg):
        return self._cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')

    def _publish_image_to_components(self, image) -> None:
        for vision_component in self._vision_components:
            vision_component.set_image(image)

    def _run_components(self, image_msg) -> None:
        for vision_component in self._vision_components:
            vision_component.run(image_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.set_logger_level("bitbots_vision", rclpy.logging.LoggingSeverity.DEBUG)
    rclpy.logging.set_logger_level("vision_yoeo", rclpy.logging.LoggingSeverity.DEBUG)
    node = YOEOVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

#! /usr/bin/env python3
from copy import deepcopy

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.experimental.events_executor import EventsExecutor
from sensor_msgs.msg import Image

from bitbots_vision.vision_modules import ros_utils, yoeo

from .params import gen

logger = rclpy.logging.get_logger("bitbots_vision")

try:
    from profilehooks import profile
except ImportError:

    def profile(func):
        return func

    logger.info("No Profiling available")


class YOEOVision(Node):
    """
    The Vision is the main ROS-node for handling all tasks related to image processing.

    This class defines the whole YOEO image processing pipeline, which uses the modules from the `vision_modules`.
    It also handles the dynamic reconfiguration of the bitbots_vision.
    """

    def __init__(self) -> None:
        super().__init__("bitbots_vision")

        logger.debug(f"Entering {self.__class__.__name__} constructor")

        self._package_path = get_package_share_directory("bitbots_vision")

        yoeo.YOEOObjectManager.set_package_directory(self._package_path)

        self._config: dict = {}
        self._cv_bridge = CvBridge()

        self._sub_image = None

        self._vision_components: list[yoeo.IVisionComponent] = []

        # Setup reconfiguration
        gen.declare_params(self)
        self.add_on_set_parameters_callback(self._dynamic_reconfigure_callback)

        # Add general params
        ros_utils.set_general_parameters(["caching"])

        # Update team color
        ros_utils.update_own_team_color(self)

        self._dynamic_reconfigure_callback(self.get_parameters_by_prefix("").values())

        logger.debug(f"Leaving {self.__class__.__name__} constructor")

    def _dynamic_reconfigure_callback(self, params) -> SetParametersResult:
        """
        Callback for the dynamic reconfigure configuration.

        :param dict params: new config
        """
        new_config = self._get_updated_config_with(params)
        self._configure_vision(new_config)
        self._config = new_config

        return SetParametersResult(successful=True)

    def _get_updated_config_with(self, params) -> dict:
        new_config = deepcopy(self._config)
        for param in params:
            new_config[param.name] = param.value
        return new_config

    def _configure_vision(self, new_config: dict) -> None:
        yoeo.YOEOObjectManager.configure(new_config)

        self._set_up_vision_components(new_config)
        self._register_subscribers(new_config)

    def _set_up_vision_components(self, new_config: dict) -> None:
        self._vision_components = []
        self._vision_components.append(yoeo.YOEOComponent())

        if new_config["component_ball_detection_active"]:
            self._vision_components.append(yoeo.BallDetectionComponent(self))
        if new_config["component_robot_detection_active"]:
            if yoeo.YOEOObjectManager.is_team_color_detection_supported():
                self._vision_components.append(yoeo.RobotDetectionComponent(self))
            else:
                self._vision_components.append(yoeo.NoTeamColorRobotDetectionComponent(self))
        if new_config["component_goalpost_detection_active"]:
            self._vision_components.append(yoeo.GoalpostDetectionComponent(self))
        if new_config["component_line_detection_active"]:
            self._vision_components.append(yoeo.LineDetectionComponent(self))
        if new_config["component_field_detection_active"]:
            self._vision_components.append(yoeo.FieldDetectionComponent(self))
        if new_config["component_debug_image_active"]:
            self._vision_components.append(yoeo.DebugImageComponent(self))

        for vision_component in self._vision_components:
            vision_component.configure(new_config, new_config["component_debug_image_active"])

    def _register_subscribers(self, config: dict) -> None:
        self._sub_image = ros_utils.create_or_update_subscriber(
            self,
            self._config,
            config,
            self._sub_image,
            "ROS_img_msg_topic",
            Image,
            callback=self._run_vision_pipeline,
        )

    @profile
    def _run_vision_pipeline(self, image_msg: Image) -> None:
        image = self._extract_image_from_message(image_msg)
        if image is None:
            logger.error("Vision pipeline - Image content is None")
            return

        self._forward_image_to_components(image)
        self._run_components(image_msg)

    def _extract_image_from_message(self, image_msg: Image) -> np.ndarray:
        return self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

    def _forward_image_to_components(self, image: np.ndarray) -> None:
        for vision_component in self._vision_components:
            vision_component.set_image(image)

    def _run_components(self, image_msg: Image) -> None:
        for vision_component in self._vision_components:
            vision_component.run(image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YOEOVision()
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()

#! /usr/bin/env python3
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.experimental.events_executor import EventsExecutor
from sensor_msgs.msg import Image

from bitbots_vision import NodeWithConfig
from bitbots_vision.vision_modules import debug, ros_utils, yoeo

logger = rclpy.logging.get_logger("bitbots_vision")

try:
    from profilehooks import profile
except ImportError:

    def profile(func):
        return func

    logger.info("No Profiling available")


class YOEOVision(NodeWithConfig):
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

        self._cv_bridge = CvBridge()

        self._sub_image = None

        self._vision_components: list[yoeo.AbstractVisionComponent] = []
        self._debug_image: Optional[debug.DebugImage] = None

        # Setup reconfiguration - now we use the generated parameter listener
        self.add_on_set_parameters_callback(self._dynamic_reconfigure_callback)

        # Add general params
        ros_utils.set_general_parameters(["caching"])

        # Update team color
        ros_utils.update_own_team_color(self)

        # Configure vision with initial parameters
        self._configure_vision_from_config()

        logger.debug(f"Leaving {self.__class__.__name__} constructor")

    def _config_to_dict(self) -> dict:
        """
        Convert the generated parameter config object to a dictionary for compatibility
        with existing code that expects dictionary access.
        """
        return {
            # Component activation parameters
            "component_ball_detection_active": self.config.component_ball_detection_active,
            "component_debug_image_active": self.config.component_debug_image_active,
            "component_field_detection_active": self.config.component_field_detection_active,
            "component_goalpost_detection_active": self.config.component_goalpost_detection_active,
            "component_line_detection_active": self.config.component_line_detection_active,
            "component_robot_detection_active": self.config.component_robot_detection_active,
            
            # ROS topic parameters
            "ROS_img_msg_topic": self.config.ROS_img_msg_topic,
            "ROS_ball_msg_topic": self.config.ROS_ball_msg_topic,
            "ROS_goal_posts_msg_topic": self.config.ROS_goal_posts_msg_topic,
            "ROS_robot_msg_topic": self.config.ROS_robot_msg_topic,
            "ROS_line_msg_topic": self.config.ROS_line_msg_topic,
            "ROS_line_mask_msg_topic": self.config.ROS_line_mask_msg_topic,
            "ROS_debug_image_msg_topic": self.config.ROS_debug_image_msg_topic,
            "ROS_field_mask_image_msg_topic": self.config.ROS_field_mask_image_msg_topic,
            
            # YOEO model parameters
            "yoeo_model_path": self.config.yoeo_model_path,
            "yoeo_nms_threshold": self.config.yoeo_nms_threshold,
            "yoeo_conf_threshold": self.config.yoeo_conf_threshold,
            "yoeo_framework": self.config.yoeo_framework,
            
            # Ball detection parameters
            "ball_candidate_rating_threshold": self.config.ball_candidate_rating_threshold,
            "ball_candidate_max_count": self.config.ball_candidate_max_count,
            
            # Caching parameter
            "caching": self.config.caching,
        }

    def _configure_vision_from_config(self) -> None:
        """
        Configure vision components using the current config.
        """
        config_dict = self._config_to_dict()
        self._configure_vision(config_dict)

    def _dynamic_reconfigure_callback(self, params) -> SetParametersResult:
        """
        Callback for the dynamic reconfigure configuration.

        :param params: list of changed parameters
        """
        # Update the config from the parameter listener
        self.config = self.param_listener.get_params()
        
        # Configure vision with the updated config
        self._configure_vision_from_config()

        return SetParametersResult(successful=True)

    def _configure_vision(self, new_config: dict) -> None:
        yoeo.YOEOObjectManager.configure(new_config)

        debug_image = debug.DebugImage(new_config["component_debug_image_active"])
        self._debug_image = debug_image

        def make_vision_component(
            component_class: type[yoeo.AbstractVisionComponent], **kwargs
        ) -> yoeo.AbstractVisionComponent:
            return component_class(
                node=self,
                yoeo_handler=yoeo.YOEOObjectManager.get(),
                debug_image=debug_image,
                config=new_config,
                **kwargs,
            )

        self._vision_components = [make_vision_component(yoeo.YOEOComponent)]

        if new_config["component_ball_detection_active"]:
            self._vision_components.append(make_vision_component(yoeo.BallDetectionComponent))
        if new_config["component_robot_detection_active"]:
            self._vision_components.append(
                make_vision_component(
                    yoeo.RobotDetectionComponent,
                    team_color_detection_supported=yoeo.YOEOObjectManager.is_team_color_detection_supported(),
                )
            )
        if new_config["component_goalpost_detection_active"]:
            self._vision_components.append(make_vision_component(yoeo.GoalpostDetectionComponent))
        if new_config["component_line_detection_active"]:
            self._vision_components.append(make_vision_component(yoeo.LineDetectionComponent))
        if new_config["component_field_detection_active"]:
            self._vision_components.append(make_vision_component(yoeo.FieldDetectionComponent))
        if new_config["component_debug_image_active"]:
            self._vision_components.append(make_vision_component(yoeo.DebugImageComponent))

        # For the subscriber update, we'll pass the last config dict or None for the first time
        old_config_dict = getattr(self, '_last_config_dict', None)
        self._sub_image = ros_utils.create_or_update_subscriber(
            self,
            old_config_dict,
            new_config,
            self._sub_image,
            "ROS_img_msg_topic",
            Image,
            callback=self._run_vision_pipeline,
        )
        # Remember this config for next time
        self._last_config_dict = new_config.copy()

    @profile
    def _run_vision_pipeline(self, image_msg: Image) -> None:
        image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

        assert self._debug_image is not None, "Debug image not initialized"
        self._debug_image.set_image(image)

        for vision_component in self._vision_components:
            vision_component.run(image, image_msg.header)


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

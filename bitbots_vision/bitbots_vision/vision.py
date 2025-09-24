#! /usr/bin/env python3
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image

from bitbots_vision.vision_modules import debug, ros_utils, yoeo
from bitbots_vision.vision_parameters import bitbots_vision as parameters

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

        # Setup parameter listener directly
        self.param_listener = parameters.ParamListener(self)
        self.config = self.param_listener.get_params()

        self._package_path = get_package_share_directory("bitbots_vision")

        yoeo.YOEOObjectManager.set_package_directory(self._package_path)

        self._cv_bridge = CvBridge()

        self._sub_image = None

        self._vision_components: list[yoeo.AbstractVisionComponent] = []
        self._debug_image: Optional[debug.DebugImage] = None

        # Setup reconfiguration callback
        self.add_on_set_parameters_callback(self._dynamic_reconfigure_callback)

        # Add general params
        ros_utils.set_general_parameters(["caching"])

        # Update team color
        ros_utils.update_own_team_color(self)

        # Configure vision with initial parameters
        self._configure_vision_from_config()

        logger.debug(f"Leaving {self.__class__.__name__} constructor")

    def _configure_vision_from_config(self) -> None:
        """
        Configure vision components using the current config.
        """
        self._configure_vision(self.config)

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

    def _configure_vision(self, config) -> None:
        yoeo.YOEOObjectManager.configure(config)

        debug_image = debug.DebugImage(config.component_debug_image_active)
        self._debug_image = debug_image

        def make_vision_component(
            component_class: type[yoeo.AbstractVisionComponent], **kwargs
        ) -> yoeo.AbstractVisionComponent:
            return component_class(
                node=self,
                yoeo_handler=yoeo.YOEOObjectManager.get(),
                debug_image=debug_image,
                config=config,  # Now passing config object directly
                **kwargs,
            )

        self._vision_components = [make_vision_component(yoeo.YOEOComponent)]

        if config.component_ball_detection_active:
            self._vision_components.append(make_vision_component(yoeo.BallDetectionComponent))
        if config.component_robot_detection_active:
            self._vision_components.append(
                make_vision_component(
                    yoeo.RobotDetectionComponent,
                    team_color_detection_supported=yoeo.YOEOObjectManager.is_team_color_detection_supported(),
                )
            )
        if config.component_goalpost_detection_active:
            self._vision_components.append(make_vision_component(yoeo.GoalpostDetectionComponent))
        if config.component_line_detection_active:
            self._vision_components.append(make_vision_component(yoeo.LineDetectionComponent))
        if config.component_field_detection_active:
            self._vision_components.append(make_vision_component(yoeo.FieldDetectionComponent))
        if config.component_debug_image_active:
            self._vision_components.append(make_vision_component(yoeo.DebugImageComponent))

        # For the subscriber update, handle the topic name directly
        old_topic = getattr(self, '_last_img_topic', None)
        current_topic = config.ROS_img_msg_topic
        
        if old_topic != current_topic:
            self._sub_image = self.create_subscription(
                Image, 
                current_topic, 
                self._run_vision_pipeline, 
                1
            )
            logger.debug(f"Registered new subscriber at {current_topic}")
            
        # Remember this topic for next time
        self._last_img_topic = current_topic

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

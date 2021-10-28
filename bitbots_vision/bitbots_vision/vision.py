#! /usr/bin/env python3

import os
import cv2
import rclpy
from rclpy import logging
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from copy import deepcopy
from cv_bridge import CvBridge
from threading import Thread, Lock
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped
from humanoid_league_msgs.msg import BallInImageArray, LineInformationInImage, \
    ObstacleInImageArray, ObstacleInImage, RegionOfInterestWithImage, \
    GoalPostInImageArray, Audio
from bitbots_vision.vision_modules import lines, field_boundary, color, debug, \
    obstacle, yolo_handler, ros_utils, candidate

logger = logging.get_logger('bitbots_vision')

try:
    from profilehooks import profile, timecall # Profilehooks profiles certain functions in you add the @profile or @timecall decorator.
except ImportError:
    logger.info("No Profiling avalabile")


class Vision(Node):
    """
    The Vision is the main ROS-node for handling all tasks related to image processing.

    This class defines the whole image processing pipeline, which uses the modules from the `vision_modules`.
    It also handles the dynamic reconfiguration of the bitbots_vision.
    """
    def __init__(self):
        # type () -> None
        """
        Initiating 'bitbots_vision' node.

        :return: None
        """

        super().__init__('bitbots_vision', automatically_declare_parameters_from_overrides=True)

        self._package_path = get_package_share_directory('bitbots_vision')

        logger.info('Initializing vision...')

        self._cv_bridge = CvBridge()

        self._config = {}

        # Publisher placeholder
        self._pub_audio = None
        self._pub_balls = None
        self._pub_lines = None
        self._pub_line_mask = None
        self._pub_obstacle = None
        self._pub_goal_posts = None
        self._pub_debug_image = None
        self._pub_convex_field_boundary = None
        self._pub_white_mask_image = None
        self._pub_red_mask_image = None
        self._pub_blue_mask_image = None
        self._pub_field_mask_image = None
        self._pub_dynamic_color_lookup_table_field_mask_image = None

        # Subscriber placeholder
        self._sub_image = None
        self._sub_dynamic_color_lookup_table_msg_topic = None

        # Debug image drawer placeholder
        self._debug_image_creator = None

        # Needed for operations that should only be executed on the first image
        self._first_image_callback = True

        # Reconfigure data transfer variable
        self._transfer_reconfigure_data = None
        self._transfer_reconfigure_data_mutex = Lock()

        # Image transfer variable
        self._transfer_image_msg = None
        self._transfer_image_msg_mutex = Lock()

        # Yolo placeholder
        self._yolo = None

        # Add model enums to _config  TODO
        # ros_utils.add_model_enums(VisionConfig, self._package_path)
        # ros_utils.add_color_lookup_table_enum(VisionConfig, self._package_path)

        self.add_on_set_parameters_callback(self._dynamic_reconfigure_callback)

        #logger.info(str(self.get_parameter('neural_network_type').value))

        # Add general params
        ros_utils.set_general_parameters(["caching"])

        self._dynamic_reconfigure_callback(self.get_parameters_by_prefix("").values())

        # Define the rate of a sleep timer
        #self._rate = self.create_rate(130)

        # Run the vision main loop
        #self._main_loop()

    def _dynamic_reconfigure_callback(self, params):
        """
        Callback for the dynamic reconfigure configuration.

        :param config: New _config
        :param level: The level is a definable int in the Vision.cfg file. All changed params are or ed together by dynamic reconfigure.
        """
        logger.info("Test")

        config = deepcopy(self._config)

        logger.info(str(params))
        # Set data
        for param in params:
            config[param.name] = param.value
        self._configure_vision(config)
        return SetParametersResult(successful=True)

    def _configure_vision(self, config):
        """
        Handle dynamic reconfigure configuration.

        :param config: New _config
        :param level: The level is a definable int in the Vision.cfg file. All changed params are or ed together by dynamic reconfigure.
        """
        self._register_or_update_all_publishers(config)

        return

        # Set max number of balls
        self._max_balls = config['ball_candidate_max_count']

        # Set some thresholds
        # Brightness threshold which determines if the camera cap is on the camera.
        self._blind_threshold = config['vision_blind_threshold']
        # Threshold for ball candidates
        self._ball_candidate_threshold = config['ball_candidate_rating_threshold']
        # Maximum offset for balls over the convex field boundary
        self._ball_candidate_y_offset = config['ball_candidate_field_boundary_y_offset']
        # Maximum offset for balls over the convex field boundary
        self._goal_post_field_boundary_y_offset = config['goal_post_field_boundary_y_offset']

        # Which line type should we publish?
        self._use_line_points = config['line_detector_use_line_points']
        self._use_line_mask = config['line_detector_use_line_mask']

        # Should the debug image be published?
        if ros_utils.config_param_change(self._config, config, 'vision_publish_debug_image'):
            if config['vision_publish_debug_image']:
                rospy.loginfo('Debug images are enabled', logger_name="vision")
            else:
                rospy.loginfo('Debug images are disabled', logger_name="vision")
            # Create debug drawer
            self._debug_image_creator = debug.DebugImage(config['vision_publish_debug_image'])

        # Should the HSV mask images be published?
        if ros_utils.config_param_change(self._config, config, 'vision_publish_HSV_mask_image'):
            self._publish_HSV_mask_image = config['vision_publish_HSV_mask_image']
            if self._publish_HSV_mask_image:
                rospy.loginfo('HSV mask image publishing is enabled', logger_name="vision_hsv_color_detector")
            else:
                rospy.loginfo('HSV mask image publishing is disabled', logger_name="vision_hsv_color_detector")

        # Should the (dynamic color lookup table-) field mask image be published?
        if ros_utils.config_param_change(self._config, config, 'vision_publish_field_mask_image'):
            self._publish_field_mask_image = config['vision_publish_field_mask_image']
            if self._publish_field_mask_image:
                rospy.loginfo('(Dynamic color lookup table-) Field mask image publishing is enabled', logger_name="dynamic_color_lookup_table")
            else:
                rospy.loginfo('(Dynamic color lookup table-) Field mask image publishing is disabled', logger_name="dynamic_color_lookup_table")

        # Set the white color detector
        if ros_utils.config_param_change(self._config, config, r'^white_color_detector_'):
            if config['white_color_detector_use_color_lookup_table']:
                self._white_color_detector = color.PixelListColorDetector(config, self._package_path, 'white_color_detector_color_lookup_table_path')
            else:
                self._white_color_detector = color.HsvSpaceColorDetector(config, "white")

        # Set the red color detector
        if ros_utils.config_param_change(self._config, config, r'^red_color_detector_'):
            self._red_color_detector = color.HsvSpaceColorDetector(config, "red")

        # Set the blue color detector
        if ros_utils.config_param_change(self._config, config, r'^blue_color_detector_'):
            self._blue_color_detector = color.HsvSpaceColorDetector(config, "blue")

        # Check if params changed
        if ros_utils.config_param_change(self._config, config,
                r'^field_color_detector_|dynamic_color_lookup_table_') and not config['field_color_detector_use_hsv']:
            # Check if the dynamic color lookup table field color detector or the static field color detector should be used
            if config['dynamic_color_lookup_table_active']:
                # Set dynamic color lookup table field color detector
                self._field_color_detector = color.DynamicPixelListColorDetector(
                    config,
                    self._package_path)
            else:
                # Unregister old subscriber
                if self._sub_dynamic_color_lookup_table_msg_topic is not None:
                    # self._sub_dynamic_color_lookup_table_msg_topic.unregister()  # Do not use this method, does not work
                    self._sub_dynamic_color_lookup_table_msg_topic = None
                # Set the static field color detector
                self._field_color_detector = color.PixelListColorDetector(
                    config,
                    self._package_path)

        # Check if params changed
        if ros_utils.config_param_change(self._config, config,
                r'^field_color_detector_|field_color_detector_use_hsv') and config['field_color_detector_use_hsv']:
            # Unregister old subscriber
            if self._sub_dynamic_color_lookup_table_msg_topic is not None:
                # self._sub_dynamic_color_lookup_table_msg_topic.unregister()  # Do not use this method, does not work
                self._sub_dynamic_color_lookup_table_msg_topic = None

            # Override field color hsv detector
            self._field_color_detector = color.HsvSpaceColorDetector(config, "field")
        # Get field boundary detector class by name from _config
        field_boundary_detector_class = field_boundary.FieldBoundaryDetector.get_by_name(
            config['field_boundary_detector_search_method'])

        # Set the field boundary detector
        self._field_boundary_detector = field_boundary_detector_class(
            config,
            self._field_color_detector)

        # Set the line detector
        self._line_detector = lines.LineDetector(
            config,
            self._white_color_detector,
            self._field_color_detector,
            self._field_boundary_detector)

        # Set the obstacle detector
        self._obstacle_detector = obstacle.ObstacleDetector(
            config,
            self._field_boundary_detector)

        # If dummy ball detection is activated, set the dummy ballfinder as ball detector
        if config['neural_network_type'] == 'dummy':
            self._ball_detector = candidate.DummyCandidateFinder()
            # If we don't use YOLO set the conventional goalpost detector.
            self._goalpost_detector = obstacle.ColorObstacleDetector(
                self._obstacle_detector,
                self._white_color_detector,
                threshold=config['obstacle_color_threshold'])

        # Check if the yolo ball/goalpost detector is activated and if the non tpu version is used
        if config['neural_network_type'] in ['yolo_opencv', 'yolo_darknet', 'yolo_pytorch']:
            if ros_utils.config_param_change(self._config, config, ['yolo_darknet_model_path', 'neural_network_type']):
                # Build absolute model path
                yolo_darknet_model_path = os.path.join(self._package_path, 'models', config['yolo_darknet_model_path'])
                # Check if it exists
                if not os.path.exists(os.path.join(yolo_darknet_model_path, "yolo_weights.weights")):
                    rospy.logerr('The specified yolo darknet model file doesn\'t exist!', logger_name="vision_yolo")
                else:
                    # Decide which yolo implementation should be used
                    if config['neural_network_type'] == 'yolo_opencv':
                        # Load OpenCV implementation (uses OpenCL)
                        self._yolo = yolo_handler.YoloHandlerOpenCV(config, yolo_darknet_model_path)
                    elif config['neural_network_type'] == 'yolo_darknet':
                        # Load Darknet implementation (uses CUDA)
                        self._yolo = yolo_handler.YoloHandlerDarknet(config, yolo_darknet_model_path)
                    elif config['neural_network_type'] == 'yolo_pytorch':
                        self._yolo = yolo_handler.YoloHandlerPytorch(config, yolo_darknet_model_path)
                    rospy.loginfo(config['neural_network_type'] + " vision is running now", logger_name="vision_yolo")

            # For other changes only modify the config
            elif ros_utils.config_param_change(self._config, config, r'yolo_'):
                self._yolo.set_config(config)

            # Set both ball and goalpost detector
            self._ball_detector = yolo_handler.YoloBallDetector(config, self._yolo)
            self._goalpost_detector = yolo_handler.YoloGoalpostDetector(config, self._yolo)
            # Check if we use the yolo robot detection
            if "robot" in self._yolo.get_classes():
                self._obstacle_detector = yolo_handler.YoloRobotDetector(config, self._yolo)

        # Check if  tpu version of yolo ball/goalpost detector is used
        if config['neural_network_type'] in ['yolo_ncs2']:
            if ros_utils.config_param_change(self._config, config, ['neural_network_type', 'yolo_openvino_model_path']):
                # Build absolute model path
                yolo_openvino_model_path = os.path.join(self._package_path, 'models', config['yolo_openvino_model_path'])
                # Check if it exists
                if not os.path.exists(os.path.join(yolo_openvino_model_path, "yolo.bin")) \
                        or not os.path.exists(os.path.join(yolo_openvino_model_path, "yolo.xml")):
                    rospy.logerr('The specified yolo openvino model file doesn\'t exist!', logger_name="vision_yolo")
                else:
                    self._yolo = yolo_handler.YoloHandlerNCS2(config, yolo_openvino_model_path)
                    rospy.loginfo(config['neural_network_type'] + " vision is running now", logger_name="vision_yolo")
            # For other changes only modify the config
            elif ros_utils.config_param_change(self._config, config, r'yolo_'):
                self._yolo.set_config(config)

            # Set both ball and goalpost detector
            self._ball_detector = yolo_handler.YoloBallDetector(config, self._yolo)
            self._goalpost_detector = yolo_handler.YoloGoalpostDetector(config, self._yolo)
            # Check if we use the yolo robot detection
            if "robot" in self._yolo.get_classes():
                self._obstacle_detector = yolo_handler.YoloRobotDetector(config, self._yolo)

        # Set the other obstacle detectors
        self._red_obstacle_detector = obstacle.ColorObstacleDetector(
            self._obstacle_detector,
            self._red_color_detector,
            threshold=config['obstacle_color_threshold'],
            subtractors=[self._goalpost_detector])
        self._blue_obstacle_detector = obstacle.ColorObstacleDetector(
            self._obstacle_detector,
            self._blue_color_detector,
            threshold=config['obstacle_color_threshold'],
            subtractors=[self._red_obstacle_detector, self._goalpost_detector])
        self._unknown_obstacle_detector = obstacle.ColorObstacleDetector(
            self._obstacle_detector,
            threshold=config['obstacle_color_threshold'],
            subtractors=[self._red_obstacle_detector, self._blue_obstacle_detector, self._goalpost_detector])

        self._register_or_update_all_subscribers(config)

        # Define Modules that should run their calculations (modules should exist, therefore its located here)
        self._conventional_modules = [
            self._field_color_detector,
            self._white_color_detector,
            self._red_color_detector,
            self._blue_color_detector,
            self._unknown_obstacle_detector,
            self._obstacle_detector,
            self._line_detector,
        ]

        # Publish Config-message (mainly for the dynamic color lookup table node)
        ros_utils.publish_vision_config(config, self._pub_config)

        # The old _config gets replaced with the new _config
        self._config = config

    def _register_or_update_all_publishers(self, config):
        # type: (dict) -> None
        """
        This method registers all publishers needed for the vision node.
        Allways create a placeholder for each publisher in init

        :param dict config: new, incoming _config
        :return: None
        """
        #self._pub_audio = ros_utils.create_or_update_publisher(self._config, config, self._pub_audio, 'ROS_audio_msg_topic', Audio, queue_size=10)
        #self._pub_balls = ros_utils.create_or_update_publisher(self._config, config, self._pub_balls, 'ROS_ball_msg_topic', BallInImageArray)
        #self._pub_lines = ros_utils.create_or_update_publisher(self._config, config, self._pub_lines, 'ROS_line_msg_topic', LineInformationInImage, queue_size=5)
        #self._pub_line_mask = ros_utils.create_or_update_publisher(self._config, config, self._pub_line_mask, 'ROS_line_mask_msg_topic', Image)
        #self._pub_obstacle = ros_utils.create_or_update_publisher(self._config, config, self._pub_obstacle, 'ROS_obstacle_msg_topic', ObstacleInImageArray, queue_size=3)
        #self._pub_goal_posts = ros_utils.create_or_update_publisher(self._config, config, self._pub_goal_posts, 'ROS_goal_posts_msg_topic', GoalPostInImageArray, queue_size=3)
        #self._pub_debug_image = ros_utils.create_or_update_publisher(self._config, config, self._pub_debug_image, 'ROS_debug_image_msg_topic', Image)
        #self._pub_convex_field_boundary = ros_utils.create_or_update_publisher(self._config, config, self._pub_convex_field_boundary, 'ROS_field_boundary_msg_topic', PolygonStamped)
        #self._pub_white_mask_image = ros_utils.create_or_update_publisher(self._config, config, self._pub_white_mask_image, 'ROS_white_HSV_mask_image_msg_topic', Image)
        #self._pub_red_mask_image = ros_utils.create_or_update_publisher(self._config, config, self._pub_red_mask_image, 'ROS_red_HSV_mask_image_msg_topic', Image)
        #self._pub_blue_mask_image = ros_utils.create_or_update_publisher(self._config, config, self._pub_blue_mask_image, 'ROS_blue_HSV_mask_image_msg_topic', Image)
        #self._pub_field_mask_image = ros_utils.create_or_update_publisher(self._config, config, self._pub_field_mask_image, 'ROS_field_mask_image_msg_topic', Image)
        #self._pub_dynamic_color_lookup_table_field_mask_image = ros_utils.create_or_update_publisher(self._config, config, self._pub_dynamic_color_lookup_table_field_mask_image, 'ROS_dynamic_color_lookup_table_field_mask_image_msg_topic', Image)

    def _register_or_update_all_subscribers(self, config):
        # type: (dict) -> None
        """
        This method registers all subscribers needed for the vision node.

        :param dict config: new, incoming _config
        :return: None
        """
        self._sub_image = ros_utils.create_or_update_subscriber(self._config, config, self._sub_image, 'ROS_img_msg_topic', Image, callback=self._image_callback, queue_size=config['ROS_img_msg_queue_size'], buff_size=60000000) # https://github.com/ros/ros_comm/issues/536

        if isinstance(self._field_color_detector, color.DynamicPixelListColorDetector):
            self._sub_dynamic_color_lookup_table_msg_topic = ros_utils.create_or_update_subscriber(self._config, config, self._sub_dynamic_color_lookup_table_msg_topic, 'ROS_dynamic_color_lookup_table_msg_topic', ColorLookupTable, callback=self._field_color_detector.color_lookup_table_callback, queue_size=1, buff_size=2 ** 20)

    def _image_callback(self, image_msg):
        # type: (Image) -> None
        """
        This method is called by the Image-message subscriber.
        Old Image-messages were dropped.

        Sometimes the queue gets to large, even when the size is limited to 1.
        That's, why we drop old images manually.
        """
        # Drops old images and cleans up the queue.
        # Still accepts very old images, that are most likely from ROS bags.
        image_age = rospy.get_rostime() - image_msg.header.stamp
        if 1.0 < image_age.to_sec() < 1000.0:
            rospy.logwarn(f"Vision: Dropped incoming Image-message, because its too old! ({image_age.to_sec()} sec)",
                            logger_name="vision")
            return

        if self._transfer_image_msg_mutex.locked():
            return

        with self._transfer_image_msg_mutex:
            # Transfer the image to the main thread
            self._transfer_image_msg = image_msg

    def _handle_image(self, image_msg):
        """
        Runs the vision pipeline

        :param image_msg: Image message provided by ROS
        """
        # converting the ROS image message to CV2-image
        image = self._cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')

        # Skip if image is None
        if image is None:
            rospy.logdebug("Image content is None :(", logger_name="vision")
            return

        # Check if its the first image callback
        if self._first_image_callback:
            # Check if a cap may be on the camera
            self._handle_forgotten_camera_cap(image)

        # Instances that should be notified with the new image
        internal_image_subscribers =[
            self._field_color_detector,
            self._white_color_detector,
            self._red_color_detector,
            self._blue_color_detector,
            self._unknown_obstacle_detector,
            self._field_boundary_detector,
            self._obstacle_detector,
            self._red_obstacle_detector,
            self._blue_obstacle_detector,
            self._goalpost_detector,
            self._line_detector,
            self._ball_detector,
            self._debug_image_creator,
        ]

        # Distribute the image to the detectors
        # Iterate over subscribers
        for vision_object in internal_image_subscribers:
            # Send image
            vision_object.set_image(image)

        # Check if the vision should run the conventional and neural net part parallel
        if self._config['vision_parallelize']:
            # Create and start threads for conventional calculation and neural net
            neural_network_thread = Thread(target=self._ball_detector.compute)

            conventional_thread = Thread(target=self._conventional_precalculation())

            conventional_thread.start()
            neural_network_thread.start()

            # Wait for both threads
            conventional_thread.join()
            neural_network_thread.join()
        else:
            # Calc conventional calculation and neural net
            self._ball_detector.compute()
            self._conventional_precalculation()

        ########
        # Ball #
        ########

        # Get a number of top balls under the field boundary, which have an high enough rating
        all_balls = self._ball_detector.get_top_candidates(count=self._max_balls)
        balls_under_field_boundary = \
            self._field_boundary_detector.candidates_under_convex_field_boundary(
                all_balls,
                self._ball_candidate_y_offset)
        top_balls = candidate.Candidate.rating_threshold(
            balls_under_field_boundary,
            self._ball_candidate_threshold)
        # check whether there are ball candidates
        if top_balls:
            # Convert ball cancidate list to ball message list
            list_of_balls = map(ros_utils.build_ball_msg, top_balls)
            # Create balls msg with the list of balls
            balls_msg = ros_utils.build_balls_msg(image_msg.header, list_of_balls)
            # Publish balls
            self._pub_balls.publish(balls_msg)

        # Debug draw all ball candidates
        self._debug_image_creator.draw_ball_candidates(
            all_balls,
            (0, 0, 255))
        # Debug draw possible ball candidates under the field boundary
        self._debug_image_creator.draw_ball_candidates(
            balls_under_field_boundary,
            (0, 255, 255))
        # Debug draw top ball candidate
        self._debug_image_creator.draw_ball_candidates(
            top_balls,
            (0, 255, 0),
            thickness=2)

        #############
        # Obstacles #
        #############

        # Init list for obstacle msgs
        list_of_obstacle_msgs = []
        # Add red obstacles
        list_of_obstacle_msgs.extend(ros_utils.build_obstacle_msgs(ObstacleInImage.ROBOT_MAGENTA,
                                                                   self._red_obstacle_detector.get_candidates()))
        # Add blue obstacles
        list_of_obstacle_msgs.extend(ros_utils.build_obstacle_msgs(ObstacleInImage.ROBOT_CYAN,
                                                                   self._blue_obstacle_detector.get_candidates()))
        # Add UFO's (Undefined Found Obstacles)
        list_of_obstacle_msgs.extend(ros_utils.build_obstacle_msgs(ObstacleInImage.ROBOT_UNDEFINED,
                                                                   self._unknown_obstacle_detector.get_candidates()))
        # Build obstacles msgs containing all obstacles
        obstacles_msg = ros_utils.build_obstacle_array_msg(image_msg.header, list_of_obstacle_msgs)
        # Publish obstacles
        self._pub_obstacle.publish(obstacles_msg)

        # Debug draw unknown obstacles
        self._debug_image_creator.draw_obstacle_candidates(
            self._unknown_obstacle_detector.get_candidates(),
            (0, 0, 0),
            thickness=3)
        # Debug draw red obstacles
        self._debug_image_creator.draw_obstacle_candidates(
            self._red_obstacle_detector.get_candidates(),
            (0, 0, 255),
            thickness=3)
        # Debug draw blue obstacles
        self._debug_image_creator.draw_obstacle_candidates(
            self._blue_obstacle_detector.get_candidates(),
            (255, 0, 0),
            thickness=3)

        ########
        # Goal #
        ########

        # Get all goalposts under field boundary
        goal_posts = self._field_boundary_detector.candidates_under_convex_field_boundary(
            self._goalpost_detector.get_candidates(),
            self._goal_post_field_boundary_y_offset)

        # Get goalpost msgs and add them to the detected goal posts list
        goal_post_msgs = ros_utils.build_goal_post_msgs(goal_posts)
        # Create goalposts msg
        goal_posts_msg = ros_utils.build_goal_post_array_msg(image_msg.header, goal_post_msgs)
        # Check if there is a goal
        if goal_posts_msg:
            # If we have a goal, lets publish it
            self._pub_goal_posts.publish(goal_posts_msg)

        # Debug draw all goal posts
        self._debug_image_creator.draw_obstacle_candidates(
            self._goalpost_detector.get_candidates(),
            (180, 180, 180),
            thickness=3)
        # Debug draw goal posts which start in the field
        self._debug_image_creator.draw_obstacle_candidates(
            goal_posts,
            (255, 255, 255),
            thickness=3)

        #########
        # Lines #
        #########
        if self._use_line_points:
            # Build a LineSegmentInImage message for each linepoint
            line_points = self._line_detector.get_linepoints()
            # Create line segments
            line_segments = ros_utils.convert_line_points_to_line_segment_msgs(line_points)
            # Create line msg
            line_msg = ros_utils.build_line_information_in_image_msg(image_msg.header, line_segments)
            # Publish lines
            self._pub_lines.publish(line_msg)

            # Draw debug line points
            self._debug_image_creator.draw_points(
                line_points,
                (0, 0, 255))

        if self._use_line_mask:
            # Define detections (Balls, Goal Posts) that are excluded from the line mask
            excluded_objects = top_balls + goal_posts
            # Get line pixel mask
            line_mask = self._line_detector.get_line_mask_without_other_objects(excluded_objects)
            # Create line mask message
            line_mask_message = ros_utils.build_image_msg(image_msg.header, line_mask, '8UC1')
            # Publish line mask
            self._pub_line_mask.publish(line_mask_message)

            # Draw debug line mask
            self._debug_image_creator.draw_mask(
                line_mask,
                color=(255, 0, 0),
                opacity=0.8)

        ##################
        # Field boundary #
        ##################

        # Get field boundary msg
        convex_field_boundary = self._field_boundary_detector.get_convex_field_boundary_points()
        # Build ros message
        convex_field_boundary_msg = ros_utils.build_field_boundary_polygon_msg(image_msg.header, convex_field_boundary)
        # Publish field boundary
        self._pub_convex_field_boundary.publish(convex_field_boundary_msg)

        # Debug draw convex field boundary
        self._debug_image_creator.draw_field_boundary(
            convex_field_boundary,
            (0, 255, 255))
        # Debug draw field boundary
        self._debug_image_creator.draw_field_boundary(
            self._field_boundary_detector.get_field_boundary_points(),
            (0, 0, 255))

        #########
        # Debug #
        #########

        # Check, if HSV mask images should be published
        if self._publish_HSV_mask_image:
            # Mask images
            white_mask = self._white_color_detector.get_mask_image()
            red_mask = self._red_color_detector.get_mask_image()
            blue_mask = self._blue_color_detector.get_mask_image()

            # Publish mask images
            self._pub_white_mask_image.publish(
                ros_utils.build_image_msg(image_msg.header, white_mask, '8UC1'))
            self._pub_red_mask_image.publish(
                ros_utils.build_image_msg(image_msg.header, red_mask, '8UC1'))
            self._pub_blue_mask_image.publish(
                ros_utils.build_image_msg(image_msg.header, blue_mask, '8UC1'))

        # Check, if field mask image should be published
        if self._publish_field_mask_image:
            if isinstance(self._field_color_detector, color.DynamicPixelListColorDetector):
                # Mask image
                dyn_field_mask = self._field_color_detector.get_mask_image()
                static_field_mask = self._field_color_detector.get_static_mask_image()
                # Publish mask image
                self._pub_dynamic_color_lookup_table_field_mask_image.publish(
                    ros_utils.build_image_msg(image_msg.header, dyn_field_mask, '8UC1'))
                self._pub_field_mask_image.publish(
                    ros_utils.build_image_msg(image_msg.header, static_field_mask, '8UC1'))
            else:
                # Mask image
                field_mask = self._field_color_detector.get_mask_image()
                # Publish mask image
                self._pub_field_mask_image.publish(
                    ros_utils.build_image_msg(image_msg.header, field_mask, '8UC1'))

        # Check if we should draw debug image
        if self._debug_image_creator.active:
            # publish debug image
            self._pub_debug_image.publish(
                ros_utils.build_image_msg(
                    image_msg.header,
                    self._debug_image_creator.get_image(),
                    'bgr8'))

    def _conventional_precalculation(self):
        """
        Starts the conventional calculations
        """
        # Run all modules
        for module in self._conventional_modules:
            module.compute()

    def _handle_forgotten_camera_cap(self, image):
        # type: (np.array) -> None
        """
        Detects a forgotten cap on the camera and notifies this via speech

        :param image: Image
        """
        # Calc the mean brightness of the image to detect a forgotten camera cap
        mean = cv2.mean(image)

        # Notify if there is a camera cap detected
        if sum(mean) < self._blind_threshold:
            rospy.logerr("Image is too dark! Camera cap not removed?", logger_name="vision")
            ros_utils.speak("Hey!   Remove my camera cap!", self._pub_audio)


def main(args=None):
    rclpy.init(args=args)
    node = Vision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

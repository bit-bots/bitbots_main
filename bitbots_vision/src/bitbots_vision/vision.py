#! /usr/bin/env python3

import os
import cv2
import rospy
import rospkg
import threading
import time
from copy import deepcopy
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from humanoid_league_msgs.msg import BallsInImage, LineInformationInImage, \
    ObstaclesInImage, ObstacleInImage, ImageWithRegionOfInterest, \
    GoalPartsInImage, FieldBoundaryInImage, Speak
from bitbots_vision.vision_modules import lines, field_boundary, color, debug, \
    fcnn_handler, live_fcnn_03, dummy_ballfinder, obstacle, yolo_handler, ros_utils
from bitbots_vision.cfg import VisionConfig
from bitbots_msgs.msg import Config, ColorSpace
try:
    from profilehooks import profile, timecall # Profilehooks profiles certain functions in you add the @profile or @timecall decorator.
except ImportError:
    rospy.loginfo("No Profiling avalabile", logger_name="vision")


class Vision:
    def __init__(self):
        # type () -> None
        """
        Vision is the main ROS-node for handling all tasks related to image processing.
        Initiating 'bitbots_vision' node.

        :return: None
        """
        rospack = rospkg.RosPack()
        self._package_path = rospack.get_path('bitbots_vision')

        rospy.init_node('bitbots_vision')
        rospy.loginfo('Initializing vision...', logger_name="vision")

        self._cv_bridge = CvBridge()

        self._config = {}

        # Publisher placeholder
        self._pub_balls = None
        self._pub_lines = None
        self._pub_obstacle = None
        self._pub_goal_parts = None
        self._pub_ball_fcnn = None
        self._pub_debug_image = None
        self._pub_debug_fcnn_image = None
        self._pub_convex_field_boundary = None
        self._pub_white_mask_image = None
        self._pub_red_mask_image = None
        self._pub_blue_mask_image = None
        self._pub_field_mask_image = None
        self._pub_dynamic_color_space_field_mask_image = None

        # Subscriber placeholder
        self._sub_image = None
        self._sub_dynamic_color_space_msg_topic = None

        self._debug_image_creator = debug.DebugImage()

        # Register static publishers
        # Register publisher of 'vision_config'-messages
        # For changes of topic name: also change topic name in dynamic_color_space.py
        self._pub_config = rospy.Publisher(
            'vision_config',
            Config,
            queue_size=1,
            latch=True)

        # Speak publisher
        self._speak_publisher = rospy.Publisher('/speak', Speak, queue_size=10)

        # Needed for operations that should only be executed on the first image
        self._first_image_callback = True

        # Reconfigure dict transfer variable
        self._transfer_reconfigure_data = None
        self._transfer_reconfigure_data_read_flag = False

        # Image transfer variable
        self._transfer_image_msg = None
        self._transfer_image_msg_read_flag = False

        # Add model enums to _config
        ros_utils.add_model_enums(VisionConfig, self._package_path)
        ros_utils.add_color_space_enums(VisionConfig, self._package_path)

        # Register VisionConfig server (dynamic reconfigure) and set callback
        srv = Server(VisionConfig, self._dynamic_reconfigure_callback)

        # Run the vision main loop
        self._main_loop()

    def _main_loop(self):
        """
        Main loop that processes the images and configuration changes
        """
        while not rospy.is_shutdown():
            # Lookup if there is another configuration available
            if self._transfer_reconfigure_data is not None:
                # Copy _config from shared memory
                self._transfer_reconfigure_data_read_flag = True
                reconfigure_data = deepcopy(self._transfer_reconfigure_data)
                self._transfer_reconfigure_data_read_flag = False
                self._transfer_reconfigure_data = None
                # Run vision reconfiguration
                self._configure_vision(*reconfigure_data)
            # Check if a new image is avalabile
            elif self._transfer_image_msg is not None:
                # Copy image from shared memory
                image_msg = deepcopy(self._transfer_image_msg)
                self._transfer_image_msg = None
                # Run the vision pipeline
                self._handle_image(image_msg)
                # Now the first image has been processed
                self._first_image_callback = False
            else:
                time.sleep(0.01)

    def _dynamic_reconfigure_callback(self, config, level):
        """
        Callback for the dynamic reconfigure configuration.

        :param config: New _config
        :param level: The level is a definable int in the Vision.cfg file. All changed params are or ed together by dynamic reconfigure.
        """
        # Check flag
        while self._transfer_reconfigure_data_read_flag and not rospy.is_shutdown():
            time.sleep(0.01)
        # Set data
        self._transfer_reconfigure_data = (config, level)

        return config


    def _configure_vision(self, config, level):
        """
        Handle dynamic reconfigure configuration.

        :param config: New _config
        :param level: The level is a definable int in the Vision.cfg file. All changed params are or ed together by dynamic reconfigure.
        """
        self._register_or_update_all_publishers(config)

        # Set some thresholds
        # Brightness threshold which determines if the camera cap is on the camera.
        self._blind_threshold = config['vision_blind_threshold']
        # Threshold for ball candidates
        self._ball_candidate_threshold = config['ball_candidate_rating_threshold']
        # Maximum offset for balls over the convex field boundary
        self._ball_candidate_y_offset = config['ball_candidate_field_boundary_y_offset']

        self._use_dynamic_color_space = config['dynamic_color_space_active']

        # Should the debug image be published?
        if ros_utils.config_param_change(self._config, config, 'vision_publish_debug_image'):
            self._publish_debug_image = config['vision_publish_debug_image']
            if self._publish_debug_image:
                rospy.loginfo('Debug images are enabled', logger_name="vision")
            else:
                rospy.loginfo('Debug images are disabled', logger_name="vision")

        # Should the fcnn output (only under the field boundary) be published?
        if ros_utils.config_param_change(self._config, config, 'ball_fcnn_publish_output'):
            self._ball_fcnn_publish_output = config['ball_fcnn_publish_output']
            if self._ball_fcnn_publish_output:
                rospy.loginfo('ball FCNN output publishing is enabled', logger_name="vision")
            else:
                rospy.loginfo('ball FCNN output publishing is disabled', logger_name="vision")

        # Should the whole fcnn output be published?
        if ros_utils.config_param_change(self._config, config, 'ball_fcnn_publish_debug_img'):
            self._publish_fcnn_debug_image = config['ball_fcnn_publish_debug_img']
            if self._publish_fcnn_debug_image:
                rospy.loginfo('Ball FCNN debug image publishing is enabled', logger_name="vision_fcnn")
            else:
                rospy.loginfo('Ball FCNN debug image publishing is disabled', logger_name="vision_fcnn")

        # Should the HSV mask images be published?
        if ros_utils.config_param_change(self._config, config, 'vision_publish_HSV_mask_image'):
            self._publish_HSV_mask_image = config['vision_publish_HSV_mask_image']
            if self._publish_HSV_mask_image:
                rospy.loginfo('HSV mask image publishing is enabled', logger_name="vision_hsv_color_detector")
            else:
                rospy.loginfo('HSV mask image publishing is disabled', logger_name="vision_hsv_color_detector")

        # Should the (dynamic color space-) field mask image be published?
        if ros_utils.config_param_change(self._config, config, 'vision_publish_field_mask_image'):
            self._publish_field_mask_image = config['vision_publish_field_mask_image']
            if self._publish_field_mask_image:
                rospy.loginfo('(Dynamic color space-) Field mask image publishing is enabled', logger_name="dynamic_color_space")
            else:
                rospy.loginfo('(Dynamic color space-) Field mask image publishing is disabled', logger_name="dynamic_color_space")

        # Print, if the vision uses the sim color or not
        if ros_utils.config_param_change(self._config, config, 'vision_use_sim_color'):
            if config['vision_use_sim_color']:
                rospy.logwarn('Loaded color space for SIMULATOR.', logger_name="vision")
            else:
                rospy.loginfo('Loaded color space for REAL WORLD.', logger_name="vision")

        # Set the white color detector
        if ros_utils.config_param_change(self._config, config, r'^white_color_detector_'):
            self._white_color_detector = color.HsvSpaceColorDetector(config, "white")

        # Set the red color detector
        if ros_utils.config_param_change(self._config, config, r'^red_color_detector_'):
            self._red_color_detector = color.HsvSpaceColorDetector(config, "red")

        # Set the blue color detector
        if ros_utils.config_param_change(self._config, config, r'^blue_color_detector_'):
            self._blue_color_detector = color.HsvSpaceColorDetector(config, "blue")

        # Check if params changed
        if ros_utils.config_param_change(self._config, config,
                r'^field_color_detector_|dynamic_color_space_|vision_use_sim_color'):
            # Check if the dynamic color space field color detector or the static field color detector should be used
            if self._use_dynamic_color_space:
                # Set dynamic color space field color detector
                self._field_color_detector = color.DynamicPixelListColorDetector(
                    config,
                    self._package_path)
            else:
                # Unregister old subscriber
                if self._sub_dynamic_color_space_msg_topic is not None:
                    self._sub_dynamic_color_space_msg_topic.unregister()
                # Set the static field color detector
                self._field_color_detector = color.PixelListColorDetector(
                    config,
                    self._package_path)

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
            self._red_color_detector,
            self._blue_color_detector,
            self._white_color_detector,
            self._field_boundary_detector)

        # Set the other obstacle detectors
        self._red_obstacle_detector = obstacle.RedObstacleDetector(self._obstacle_detector)
        self._blue_obstacle_detector = obstacle.BlueObstacleDetector(self._obstacle_detector)
        self._unknown_obstacle_detector = obstacle.UnknownObstacleDetector(self._obstacle_detector)

        # If dummy ball detection is activated, set the dummy ballfinder as ball detector
        if config['neural_network_type'] == 'dummy':
            self._ball_detector = dummy_ballfinder.DummyBallDetector()
            # If we don't use YOLO set the conventional goalpost detector.
            self._goalpost_detector = obstacle.WhiteObstacleDetector(self._obstacle_detector)

        # Check if the fcnn is activated
        if config['neural_network_type'] == 'fcnn':
            # Check if its the first callback, the fcnn is newly activated or the model has changed
            if ros_utils.config_param_change(self._config, config, ['fcnn_model_path', 'neural_network_type']):
                # Build absolute model path
                ball_fcnn_path = os.path.join(self._package_path, 'models', config['fcnn_model_path'])
                # Check if it exists
                if not os.path.exists(os.path.join(ball_fcnn_path, "model_final.index")):
                    rospy.logerr('AAAAHHHH! The specified fcnn model file doesn\'t exist! Maybe its a YOLO model? Look twice.', logger_name="vision_fcnn")
                else:
                    self._ball_fcnn = live_fcnn_03.FCNN03(ball_fcnn_path)
                    rospy.loginfo("FCNN vision is running now", logger_name="vision_fcnn")
            #Check if ball_fcnn _config or the neural network type has changed
            if ros_utils.config_param_change(self._config, config, r'^ball_fcnn_') or \
                    ros_utils.config_param_change(self._config, config, 'neural_network_type'):
                # Set fcnn handler
                self._ball_detector = fcnn_handler.FcnnHandler(
                    config,
                    self._ball_fcnn)
                # If we don't use YOLO set the conventional goalpost detector.
                self._goalpost_detector = obstacle.WhiteObstacleDetector(self._obstacle_detector)

        # Check if the yolo ball/goalpost detector is activated. No matter which implementation is used.
        if config['neural_network_type'] in ['yolo_opencv', 'yolo_darknet']:
            if ros_utils.config_param_change(self._config, config, ['yolo_model_path', 'neural_network_type']):
                # Build absolute model path
                yolo_model_path = os.path.join(self._package_path, 'models', config['yolo_model_path'])
                # Check if it exists
                if not os.path.exists(os.path.join(yolo_model_path, "yolo_weights.weights")):
                    rospy.logerr('AAAAHHHH! The specified yolo model file doesn\'t exist! Maybe its an fcnn model?', logger_name="vision_yolo")
                else:
                    # Decide which yolo implementation should be used
                    if config['neural_network_type'] == 'yolo_opencv':
                        # Load OpenCV implementation (uses OpenCL)
                        self._yolo = yolo_handler.YoloHandlerOpenCV(config, yolo_model_path)
                    elif config['neural_network_type'] == 'yolo_darknet':
                        # Load Darknet implementation (uses CUDA)
                        self._yolo = yolo_handler.YoloHandlerDarknet(config, yolo_model_path)
                    # Set both ball and goalpost detector
                    self._ball_detector = yolo_handler.YoloBallDetector(config, self._yolo)
                    self._goalpost_detector = yolo_handler.YoloGoalpostDetector(config, self._yolo)
                    rospy.loginfo(config['neural_network_type'] + " vision is running now", logger_name="vision_yolo")

        self._register_or_update_all_subscribers(config)

        # Define Modules that should run their calculations (modules should exist, theirfore its located here)
        self._conventional_modules = [
            self._obstacle_detector,
            self._line_detector,
        ]

        # Publish Config-message (mainly for the dynamic color space node)
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
        self._pub_balls = ros_utils.create_or_update_publisher(self._config, config, self._pub_balls, 'ROS_ball_msg_topic', BallsInImage)
        self._pub_lines = ros_utils.create_or_update_publisher(self._config, config, self._pub_lines, 'ROS_line_msg_topic', LineInformationInImage, queue_size=5)
        self._pub_obstacle = ros_utils.create_or_update_publisher(self._config, config, self._pub_obstacle, 'ROS_obstacle_msg_topic', ObstaclesInImage, queue_size=3)
        self._pub_goal_parts = ros_utils.create_or_update_publisher(self._config, config, self._pub_goal_parts, 'ROS_goal_parts_msg_topic', GoalPartsInImage, queue_size=3)
        self._pub_ball_fcnn = ros_utils.create_or_update_publisher(self._config, config, self._pub_ball_fcnn, 'ROS_fcnn_img_msg_topic', ImageWithRegionOfInterest)
        self._pub_debug_image = ros_utils.create_or_update_publisher(self._config, config, self._pub_debug_image, 'ROS_debug_image_msg_topic', Image)
        self._pub_convex_field_boundary = ros_utils.create_or_update_publisher(self._config, config, self._pub_convex_field_boundary, 'ROS_field_boundary_msg_topic', FieldBoundaryInImage)
        self._pub_debug_fcnn_image = ros_utils.create_or_update_publisher(self._config, config, self._pub_debug_fcnn_image, 'ROS_debug_fcnn_image_msg_topic', Image)
        self._pub_white_mask_image = ros_utils.create_or_update_publisher(self._config, config, self._pub_white_mask_image, 'ROS_white_HSV_mask_image_msg_topic', Image)
        self._pub_red_mask_image = ros_utils.create_or_update_publisher(self._config, config, self._pub_red_mask_image, 'ROS_red_HSV_mask_image_msg_topic', Image)
        self._pub_blue_mask_image = ros_utils.create_or_update_publisher(self._config, config, self._pub_blue_mask_image, 'ROS_blue_HSV_mask_image_msg_topic', Image)
        self._pub_field_mask_image = ros_utils.create_or_update_publisher(self._config, config, self._pub_field_mask_image, 'ROS_field_mask_image_msg_topic', Image)
        self._pub_dynamic_color_space_field_mask_image = ros_utils.create_or_update_publisher(self._config, config, self._pub_dynamic_color_space_field_mask_image, 'ROS_dynamic_color_space_field_mask_image_msg_topic', Image)

    def _register_or_update_all_subscribers(self, config):
        # type: (dict) -> None
        """
        This method registers all subscribers needed for the vision node.

        :param dict config: new, incoming _config
        :return: None
        """
        self._sub_image = ros_utils.create_or_update_subscriber(self._config, config, self._sub_image, 'ROS_img_msg_topic', Image, callback=self._image_callback, queue_size=config['ROS_img_msg_queue_size'], buff_size=60000000) # https://github.com/ros/ros_comm/issues/536

        if self._use_dynamic_color_space:
            self._sub_dynamic_color_space_msg_topic = ros_utils.create_or_update_subscriber(self._config, config, self._sub_dynamic_color_space_msg_topic, 'ROS_dynamic_color_space_msg_topic', ColorSpace, callback=self._field_color_detector.color_space_callback, queue_size=1, buff_size=2 ** 20)

    def _image_callback(self, image_msg):
        # type: (Image) -> None
        """
        This method is called by the Image-message subscriber.
        Old Image-messages were dropped.

        Sometimes the queue gets to large, even when the size is limited to 1.
        That's, why we drop old images manually.
        """
        # drops old images and cleans up queue. Still accepts very old images, that are most likely from ros bags.
        image_age = rospy.get_rostime() - image_msg.header.stamp
        if 1.0 < image_age.to_sec() < 1000.0:
            rospy.logwarn('Vision: Dropped incoming Image-message, because its too old! ({} sec)'.format(image_age.to_sec()),
                          logger_throttle=2, logger_name="")
            return

        # Check flag
        if self._transfer_image_msg_read_flag:
            return

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
            fcnn_thread = threading.Thread(target=self._ball_detector.compute)

            conventional_thread = threading.Thread(target=self._conventional_precalculation())

            conventional_thread.start()
            fcnn_thread.start()

            # Wait for both threads
            conventional_thread.join()
            fcnn_thread.join()
        else:
            # Calc conventional calculation and neural net
            self._ball_detector.compute()
            self._conventional_precalculation()

        ########
        # Ball #
        ########

        # Grab ball candidates from ball detector
        top_ball_candidate = self._ball_detector.get_top_ball_under_convex_field_boundary(self._field_boundary_detector, self._ball_candidate_y_offset)
        # check whether ball candidates are over rating threshold
        if top_ball_candidate and top_ball_candidate.get_rating() > self._ball_candidate_threshold:
            # Build the ball message which will be embedded in the balls message
            ball_msg = ros_utils.build_ball_msg(top_ball_candidate)
            # Create a list of balls, currently only containing the top candidate
            list_of_balls = [ball_msg]
            # Create balls msg with the list of balls
            balls_msg = ros_utils.build_balls_msg(image_msg.header, list_of_balls)
            # Publish balls
            self._pub_balls.publish(balls_msg)

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
        list_of_obstacle_msgs.extend(ros_utils.build_obstacle_msgs(ObstacleInImage.UNDEFINED,
                                                                   self._unknown_obstacle_detector.get_candidates()))
        # Build obstacles msgs containing all obstacles
        obstacles_msg = ros_utils.build_obstacles_msg(image_msg.header, list_of_obstacle_msgs)
        # Publish obstacles
        self._pub_obstacle.publish(obstacles_msg)

        ########
        # Goal #
        ########

        # Get goalpost msgs and add them to the detected goal parts list
        goal_parts = ros_utils.build_goalpost_msgs(self._goalpost_detector.get_candidates())
        # Create goalparts msg
        goal_parts_msg = ros_utils.build_goal_parts_msg(image_msg.header, goal_parts)
        # Check if there is a goal
        if goal_parts_msg:
            # If we have a goal, lets publish it
            self._pub_goal_parts.publish(goal_parts_msg)

        #########
        # Lines #
        #########

        # Build a LineSegmentInImage message for each linepoint
        line_points = self._line_detector.get_linepoints()
        # Create line segments
        line_segments = ros_utils.convert_line_points_to_line_segment_msgs(line_points)
        # Create line msg
        line_msg = ros_utils.build_line_information_in_image_msg(image_msg.header, line_segments)
        # Publish lines
        self._pub_lines.publish(line_msg)

        ##################
        # Field boundary #
        ##################

        # Get field boundary msg
        convex_field_boundary = self._field_boundary_detector.get_convex_field_boundary_points()
        # Build ros message
        convex_field_boundary_msg = ros_utils.build_field_boundary_msg(image_msg.header, convex_field_boundary)
        # Publish field boundary
        self._pub_convex_field_boundary.publish(convex_field_boundary_msg)

        #########
        # Debug #
        #########

        if self._config['neural_network_type'] == 'fcnn':
            # Publish fcnn output for the region of interest under the field boundary (for the world model)
            if self._ball_fcnn_publish_output:
                roi_msg = ros_utils.build_fcnn_region_of_interest(
                    self._ball_detector.get_fcnn_output(),
                    self._field_boundary_detector,
                    image_msg.header,
                    self._config['ball_fcnn_publish_field_boundary_offset'])
                self._pub_ball_fcnn.publish(roi_msg)

            # Publish whole fcnn output for debug purposes
            if self._publish_fcnn_debug_image:
                self._pub_debug_fcnn_image.publish(self._ball_detector.get_debug_image())

        # Check, if HSV mask images should be published
        if self._publish_HSV_mask_image:
            # Mask images
            white_mask = self._white_color_detector.get_mask_image()
            red_mask = self._red_color_detector.get_mask_image()
            blue_mask = self._blue_color_detector.get_mask_image()

            # Publish mask images
            self._pub_white_mask_image.publish(self._cv_bridge.cv2_to_imgmsg(white_mask, '8UC1'))
            self._pub_red_mask_image.publish(self._cv_bridge.cv2_to_imgmsg(red_mask, '8UC1'))
            self._pub_blue_mask_image.publish(self._cv_bridge.cv2_to_imgmsg(blue_mask, '8UC1'))

        # Check, if field mask image should be published
        if self._publish_field_mask_image:
            if self._use_dynamic_color_space:
                # Mask image
                dyn_field_mask = self._field_color_detector.get_mask_image()
                static_field_mask = self._field_color_detector.get_static_mask_image()
                # Publish mask image
                self._pub_dynamic_color_space_field_mask_image.publish(self._cv_bridge.cv2_to_imgmsg(dyn_field_mask, '8UC1'))
                self._pub_field_mask_image.publish(self._cv_bridge.cv2_to_imgmsg(static_field_mask, '8UC1'))
            else:
                # Mask image
                field_mask = self._field_color_detector.get_mask_image()
                # Publish mask image
                self._pub_field_mask_image.publish(self._cv_bridge.cv2_to_imgmsg(field_mask, '8UC1'))

        # Check if we should draw debug image
        if self._publish_debug_image:
            # Draw debug image
            debug_image = self._create_debug_image(image)
            # publish debug image
            self._pub_debug_image.publish(self._cv_bridge.cv2_to_imgmsg(debug_image, 'bgr8'))

    def _create_debug_image(self, image):
        """
        Draws a debug image

        :param image: untouched image
        :return: image with debug annotations
        """
        # Draw unknown obstacles
        self._debug_image_creator.draw_obstacle_candidates(
            self._unknown_obstacle_detector.get_candidates(),
            (0, 0, 0),
            thickness=3
        )
        # Draw red obstacles
        self._debug_image_creator.draw_obstacle_candidates(
            self._red_obstacle_detector.get_candidates(),
            (0, 0, 255),
            thickness=3
        )
        # Draw blue obstacles
        self._debug_image_creator.draw_obstacle_candidates(
            self._blue_obstacle_detector.get_candidates(),
            (255, 0, 0),
            thickness=3
        )
        # Draw goal posts
        self._debug_image_creator.draw_obstacle_candidates(
            self._goalpost_detector.get_candidates(),
            (255, 255, 255),
            thickness=3
        )
        # Draw field boundary
        self._debug_image_creator.draw_field_boundary(
            self._field_boundary_detector.get_field_boundary_points(),
            (0, 0, 255)
        )
        # Draw convex field boundary
        self._debug_image_creator.draw_field_boundary(
            self._field_boundary_detector.get_convex_field_boundary_points(),
            (0, 255, 255)
        )
        # Draw all ball candidates
        self._debug_image_creator.draw_ball_candidates(
            self._ball_detector.get_candidates(),
            (0, 0, 255)
        )
        # Draw possible ball candidates under the field boundary
        self._debug_image_creator.draw_ball_candidates(
            self._ball_detector.get_sorted_top_balls_under_convex_field_boundary(
                self._field_boundary_detector,
                self._ball_candidate_y_offset),
            (0, 255, 255)
        )
        # Draw top ball candidate
        self._debug_image_creator.draw_ball_candidates(
            [self._ball_detector.get_top_ball_under_convex_field_boundary(
                self._field_boundary_detector,
                self._ball_candidate_y_offset)],
            (0, 255, 0)
        )
        # Draw line points
        self._debug_image_creator.draw_points(
            self._line_detector.get_linepoints(),
            (0, 0, 255)
        )
        # Return image from the debug image drawer
        return self._debug_image_creator.get_image()

    def _conventional_precalculation(self):
        """
        Starts the conventional calculations
        """
        # Modules that should run their calculations
        # TODO: move this to DynReconf and add empty list to init
        self._conventional_modules = [
            self._field_color_detector,
            self._white_color_detector,
            self._red_color_detector,
            self._blue_color_detector,
            self._obstacle_detector,
            self._line_detector,
        ]
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
            ros_utils.speak("Hey!   Remove my camera cap!", self._speak_publisher)


if __name__ == '__main__':
    Vision()

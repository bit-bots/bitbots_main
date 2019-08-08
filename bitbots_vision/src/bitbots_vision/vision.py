#! /usr/bin/env python3

import os
import sys
import cv2
import yaml
import rospy
import rospkg
import threading
from profilehooks import profile, timecall
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from humanoid_league_msgs.msg import BallsInImage, LineInformationInImage, \
    ObstaclesInImage, ObstacleInImage, ImageWithRegionOfInterest, \
    GoalInImage, FieldBoundaryInImage, Speak
from bitbots_vision.vision_modules import lines, field_boundary, color, debug, \
    fcnn_handler, live_fcnn_03, dummy_ballfinder, obstacle, yolo_handler, ros_utils
from bitbots_vision.cfg import VisionConfig
from bitbots_msgs.msg import Config, ColorSpace


class Vision:
    def __init__(self):
        # type () -> None
        """
        Vision is the main ROS-node for handling all tasks related to image processing.
        Initiating 'bitbots_vision' node.

        :return: None
        """
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('bitbots_vision')

        rospy.init_node('bitbots_vision')
        rospy.loginfo('Initializing vision...')

        self.bridge = CvBridge()

        self.config = {}

        # Publisher placeholder
        self.pub_balls = None
        self.pub_lines = None
        self.pub_obstacle = None
        self.pub_goal = None
        self.pub_ball_fcnn = None
        self.pub_debug_image = None
        self.pub_debug_fcnn_image = None
        self.pub_convex_field_boundary = None
        self.pub_field_mask_image = None
        self.pub_dynamic_color_space_field_mask_image = None

        # Subscriber placeholder
        self.sub_image = None
        self.sub_dynamic_color_space_msg_topic = None

        self.debug_image_drawer = debug.DebugImage()

        # Register static publishers
        # Register publisher of 'vision_config'-messages
        # For changes of topic name: also change topic name in dynamic_color_space.py
        self.pub_config = rospy.Publisher(
            'vision_config',
            Config,
            queue_size=1,
            latch=True)

        # Speak publisher
        self.speak_publisher = rospy.Publisher('/speak', Speak, queue_size=10)

        # Needed for operations that should only be executed on the first image
        self._first_image_callback = True

        # Needed to determine whether reconfiguration is currently in progress or not
        self.reconfigure_active = False

        # Add model enums to config
        ros_utils.ROS_Utils.add_model_enums(VisionConfig, self.package_path)
        ros_utils.ROS_Utils.add_color_space_enums(VisionConfig, self.package_path)

        # Register VisionConfig server (dynamic reconfigure) and set callback
        srv = Server(VisionConfig, self._dynamic_reconfigure_callback)

        rospy.spin()

    def _dynamic_reconfigure_callback(self, config, level):
        """
        Callback for the dynamic reconfigure configuration. This callback also gets calles for the inertial configuration.
        :param config: New config
        :param level: Custom defineable value
        """
        # Deactivates Vision temporarally
        self.reconfigure_active = True

        self._register_or_update_all_publishers(config)

        # Set some thresholds
        # Brightness threshold which determins if the camera cap is on the camera.
        self._blind_threshold = config['vision_blind_threshold']
        # Threshold for ball candidates
        self._ball_candidate_threshold = config['vision_ball_candidate_rating_threshold']
        # Maximum offset for balls over the convex field boundary
        self._ball_candidate_y_offset = config['vision_ball_candidate_field_boundary_y_offset']

        # Should the debug image be published?
        if ros_utils.ROS_Utils.config_param_change(self.config, config, 'vision_publish_debug_image'):
            self.publish_debug_image = config['vision_publish_debug_image']
            if self.publish_debug_image:
                rospy.logwarn('Debug images are enabled')
            else:
                rospy.loginfo('Debug images are disabled')

        # Should the fcnn output (only under the field boundary) be published?
        if ros_utils.ROS_Utils.config_param_change(self.config, config, 'ball_fcnn_publish_output'):
            self.ball_fcnn_publish_output = config['ball_fcnn_publish_output']
            if self.ball_fcnn_publish_output:
                rospy.logwarn('ball FCNN output publishing is enabled')
            else:
                rospy.logwarn('ball FCNN output publishing is disabled')

        # Should the whole fcnn output be published?
        if ros_utils.ROS_Utils.config_param_change(self.config, config, 'ball_fcnn_publish_debug_img'):
            self.publish_fcnn_debug_image = config['ball_fcnn_publish_debug_img']
            if self.ball_fcnn_publish_output:
                rospy.logwarn('Ball FCNN debug image publishing is enabled')
            else:
                rospy.loginfo('Ball FCNN debug image publishing is disabled')

        # Print if the vision uses the sim color or not
        if ros_utils.ROS_Utils.config_param_change(self.config, config, 'vision_use_sim_color'):
            if config['vision_use_sim_color']:
                rospy.logwarn('Loaded color space for SIMULATOR.')
            else:
                rospy.loginfo('Loaded color space for REAL WORLD.')

        # Set the white color detector
        if ros_utils.ROS_Utils.config_param_change(self.config, config, r'^white_color_detector_'):
            self.white_color_detector = color.HsvSpaceColorDetector(config, "white")

        # Set the red color detector
        if ros_utils.ROS_Utils.config_param_change(self.config, config, r'^red_color_detector_'):
            self.red_color_detector = color.HsvSpaceColorDetector(config, "red")

        # Set the blue color detector
        if ros_utils.ROS_Utils.config_param_change(self.config, config, r'^blue_color_detector_'):
            self.blue_color_detector = color.HsvSpaceColorDetector(config, "blue")

        # Check if the dynamic color space field color detector or the static field color detector should be used
        if config['dynamic_color_space_active']:
            # Set dynamic color space field color detector
            self.field_color_detector = color.DynamicPixelListColorDetector(
                self.package_path,
                config,
                self.pub_field_mask_image,
                self.pub_dynamic_color_space_field_mask_image)
        else:
            # Set the static field color detector
            self.field_color_detector = color.PixelListColorDetector(
                self.package_path,
                self.pub_field_mask_image,
                config)

        # Get field boundary detector class by name from config
        field_boundary_detector_class = field_boundary.FieldBoundaryDetector.get_by_name(
            config['field_boundary_detector_search_method'])

        # Set the field boundary detector
        self.field_boundary_detector = field_boundary_detector_class(
            self.field_color_detector,
            config)

        # Set the line detector
        self.line_detector = lines.LineDetector(
            self.white_color_detector,
            self.field_color_detector,
            self.field_boundary_detector,
            config)

        # Set the obstacle detector
        self.obstacle_detector = obstacle.ObstacleDetector(
            self.red_color_detector,
            self.blue_color_detector,
            self.white_color_detector,
            self.field_boundary_detector,
            config)

        # If we don't use YOLO set the conventional goalpost detector.
        if not config['vision_ball_detector'] in ['yolo_opencv', 'yolo_darknet']:
            self.goalpost_detector = obstacle.WhiteObstacleDetector(self.obstacle_detector)
        # Set the other obstacle detectors
        self.red_obstacle_detector = obstacle.RedObstacleDetector(self.obstacle_detector)
        self.blue_obstacle_detector = obstacle.BlueObstacleDetector(self.obstacle_detector)
        self.unknown_obstacle_detector = obstacle.UnknownObstacleDetector(self.obstacle_detector)

        # Set up ball config for fcnn
        # These config params have domain-specific names which could be problematic for fcnn handlers handling e.g. goal candidates
        # This enables support for several FCNNs with different configs.
        self.ball_fcnn_config = {
            'debug': config['ball_fcnn_publish_debug_img'],
            'threshold': config['ball_fcnn_threshold'],
            'expand_stepsize': config['ball_fcnn_expand_stepsize'],
            'pointcloud_stepsize': config['ball_fcnn_pointcloud_stepsize'],
            'shuffle_candidate_list': config['ball_fcnn_shuffle_candidate_list'],
            'min_candidate_diameter': config['ball_fcnn_min_ball_diameter'],
            'max_candidate_diameter': config['ball_fcnn_max_ball_diameter'],
            'candidate_refinement_iteration_count': config['ball_fcnn_candidate_refinement_iteration_count'],
            'publish_field_boundary_offset': config['ball_fcnn_publish_field_boundary_offset'],
        }

        # If dummy ball detection is activated, set the dummy ballfinder as ball detector
        if config['vision_ball_detector'] == 'dummy':
            self.ball_detector = dummy_ballfinder.DummyClassifier(None, None)

        # Check if the fcnn ball detector is activated
        if config['vision_ball_detector'] == 'fcnn':
            # Check if its the first callback, the fcnn is newly activated or the model has changed
            if ros_utils.ROS_Utils.config_param_change(self.config, config, ['fcnn_model_path', 'vision_ball_detector']):
                # Build absolute model path
                ball_fcnn_path = os.path.join(self.package_path, 'models', config['fcnn_model_path'])
                # Check if it exists
                if not os.path.exists(os.path.join(ball_fcnn_path, "model_final.index")):
                    rospy.logerr('AAAAHHHH! The specified fcnn model file doesn\'t exist! Maybe its a YOLO model? Look twice.')
                else:
                    self.ball_fcnn = live_fcnn_03.FCNN03(ball_fcnn_path)
                    rospy.loginfo("FCNN vision is running now")
            #Check if ball_fcnn or vision_ball config has changed
            if ros_utils.ROS_Utils.config_param_change(self.config, config, r'^(ball_fcnn_)|(vision_ball_)'):
                self.ball_detector = fcnn_handler.FcnnHandler(
                    self.ball_fcnn,
                    self.field_boundary_detector,
                    self.ball_fcnn_config)

        # Check if the yolo ball/goalpost detector is activated. No matter which implementation is used.
        if config['vision_ball_detector'] in ['yolo_opencv', 'yolo_darknet']:
            if ros_utils.ROS_Utils.config_param_change(self.config, config, ['yolo_model_path', 'vision_ball_detector']):
                # Build absolute model path
                yolo_model_path = os.path.join(self.package_path, 'models', config['yolo_model_path'])
                # Check if it exists
                if not os.path.exists(os.path.join(yolo_model_path, "yolo_weights.weights")):
                    rospy.logerr('AAAAHHHH! The specified yolo model file doesn\'t exist! Maybe its an fcnn model?')
                else:
                    # Decide which yolo implementation should be used
                    if config['vision_ball_detector'] == 'yolo_opencv':
                        # Load OpenCV implementation (uses OpenCL)
                        yolo = yolo_handler.YoloHandlerOpenCV(config, yolo_model_path)
                    elif config['vision_ball_detector'] == 'yolo_darknet':
                        # Load Darknet implementation (uses CUDA)
                        yolo = yolo_handler.YoloHandlerDarknet(config, yolo_model_path)
                    # Set both ball and goalpost detector
                    self.ball_detector = yolo_handler.YoloBallDetector(yolo)
                    self.goalpost_detector = yolo_handler.YoloGoalpostDetector(yolo)
                    rospy.loginfo(config['vision_ball_detector'] + " vision is running now")

        self._register_or_update_all_subscribers(config)

        # Publish Config-message (mainly for the dynamic color space node)
        ros_utils.ROS_Utils.publish_vision_config(config, self.pub_config)

        # The old config gets replaced with the new config
        self.config = config

        # Activate Vision again
        self.reconfigure_active = False

        return config

    def _register_or_update_all_publishers(self, config):
        # type: (dict) -> None
        """
        This method registers all publishers needed for the vision node.

        :param dict config: new, incoming config
        :return: None
        """
        self.pub_balls = ros_utils.ROS_Utils.create_or_update_publisher(self.config, config, self.pub_balls, 'ROS_ball_msg_topic', BallsInImage)
        self.pub_lines = ros_utils.ROS_Utils.create_or_update_publisher(self.config, config, self.pub_lines, 'ROS_line_msg_topic', LineInformationInImage, queue_size=5)
        self.pub_obstacle = ros_utils.ROS_Utils.create_or_update_publisher(self.config, config, self.pub_obstacle, 'ROS_obstacle_msg_topic', ObstaclesInImage, queue_size=3)
        self.pub_goal = ros_utils.ROS_Utils.create_or_update_publisher(self.config, config, self.pub_goal, 'ROS_goal_msg_topic', GoalInImage, queue_size=3)
        self.pub_ball_fcnn = ros_utils.ROS_Utils.create_or_update_publisher(self.config, config, self.pub_ball_fcnn, 'ROS_fcnn_img_msg_topic', ImageWithRegionOfInterest)
        self.pub_debug_image = ros_utils.ROS_Utils.create_or_update_publisher(self.config, config, self.pub_debug_image, 'ROS_debug_image_msg_topic', Image)
        self.pub_convex_field_boundary = ros_utils.ROS_Utils.create_or_update_publisher(self.config, config, self.pub_convex_field_boundary, 'ROS_field_boundary_msg_topic', FieldBoundaryInImage)
        self.pub_debug_fcnn_image = ros_utils.ROS_Utils.create_or_update_publisher(self.config, config, self.pub_debug_fcnn_image, 'ROS_debug_fcnn_image_msg_topic', Image)
        self.pub_field_mask_image = ros_utils.ROS_Utils.create_or_update_publisher(self.config, config, self.pub_field_mask_image, 'ROS_field_mask_image_msg_topic', Image)
        self.pub_dynamic_color_space_field_mask_image = ros_utils.ROS_Utils.create_or_update_publisher(self.config, config, self.pub_dynamic_color_space_field_mask_image, 'ROS_dynamic_color_space_field_mask_image_msg_topic', Image)

    def _register_or_update_all_subscribers(self, config):
        # type: (dict) -> None
        """
        This method registers all subscribers needed for the vision node.

        :param dict config: new, incoming config
        :return: None
        """
        self.sub_image = ros_utils.ROS_Utils.create_or_update_subscriber(self.config, config, self.sub_image, 'ROS_img_msg_topic', Image, callback=self._image_callback, queue_size=config['ROS_img_queue_size'], buff_size=60000000)
        self.sub_dynamic_color_space_msg_topic = ros_utils.ROS_Utils.create_or_update_subscriber(self.config, config, self.sub_dynamic_color_space_msg_topic, 'ROS_dynamic_color_space_msg_topic', ColorSpace, callback=self.field_color_detector.color_space_callback, queue_size=1, buff_size=2**20)

    def _image_callback(self, image_msg):
        # type: (Image) -> None
        """
        This method is called by the Image-message subscriber.
        Old Image-messages were dropped.

        Sometimes the queue gets to large, even when the size is limeted to 1.
        That's, why we drop old images manually.
        """
        # drops old images and cleans up queue. Still accepts very old images, that are most likely from ros bags.
        image_age = rospy.get_rostime() - image_msg.header.stamp
        if 1.0 < image_age.to_sec() < 1000.0:
            rospy.logwarn_throttle(2, 'Vision: Dropped incoming Image-message')
            return

        # Do not process images if reconfiguration is in progress
        if self.reconfigure_active:
            rospy.loginfo("Dropped image due to dynamic reconfigure callback!")
            return

        # Catch type errors that occur during reconfiguration :(
        try:
            self.handle_image(image_msg)
            # Now this is not the first image callback anymore
            self._first_image_callback = False
        except (TypeError, cv2.error):
            if not self.reconfigure_active:
                raise
            else:
                rospy.loginfo("Dropped image due to dynamic reconfigure callback!")

    def handle_image(self, image_msg):
        """
        Runs the vision pipeline
        :param image_msg: Image message provided by ROS
        """
        # converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')

        # Skip if image is None
        if image is None:
            return

        # Check if its the first image callback
        if self._first_image_callback:
            # Check if a cap may be on the camera
            self._handle_forgotten_camera_cap(image)

        # Instances that should be notified with the new image
        internal_image_subscribers =[
            self.field_boundary_detector,
            self.obstacle_detector,
            self.red_obstacle_detector,
            self.blue_obstacle_detector,
            self.goalpost_detector,
            self.line_detector,
            self.ball_detector,
        ]

        # distribute the image to the detectors
        self._distribute_images(image, internal_image_subscribers)

        # Check if the vision should run the conventional and neural net part parrall
        if self.config['vision_parallelize']:
            # Create and start threads for conventional calculation and neural net
            fcnn_thread = threading.Thread(target=self.ball_detector.compute_top_candidate)
            conventional_thread = threading.Thread(target=self._conventional_precalculation())

            conventional_thread.start()
            fcnn_thread.start()

            # Wait for both threads
            conventional_thread.join()
            fcnn_thread.join()
        else:
            # Calc conventional calculation and neural net
            self.ball_detector.compute_top_candidate()
            self._conventional_precalculation()

        # Grab ball candidates from ball detector
        self.top_ball_candidate = self.ball_detector.get_top_ball_under_convex_field_boundary(self.field_boundary_detector)

        # check whether ball candidates are over rating threshold
        if self.top_ball_candidate and self.top_ball_candidate.get_rating() > self._ball_candidate_threshold:
            # Build the ball message which will be embedded in the balls message
            ball_msg = ros_utils.ROS_Utils.build_ball_msg(self.top_ball_candidate)
            # Create a list of balls, currently only containing the top candidate
            list_of_balls = [ball_msg]
            # Create balls msg with the list of balls
            balls_msg = ros_utils.ROS_Utils.build_balls_msg(image_msg.header, list_of_balls)
            # Publish balls
            self.pub_balls.publish(balls_msg)

        # Init list for obstacle msgs
        list_of_obstacle_msgs = []
        # Add red obstacles
        list_of_obstacle_msgs.extend(ros_utils.ROS_Utils.build_obstacle_msgs(ObstacleInImage.ROBOT_MAGENTA,
            self.red_obstacle_detector.get_candidates()))
        # Add blue obstacles
        list_of_obstacle_msgs.extend(ros_utils.ROS_Utils.build_obstacle_msgs(ObstacleInImage.ROBOT_CYAN,
            self.blue_obstacle_detector.get_candidates()))
        # Add UFO's (Undefined Found Obstacles)
        list_of_obstacle_msgs.extend(ros_utils.ROS_Utils.build_obstacle_msgs(ObstacleInImage.UNDEFINED,
            self.unknown_obstacle_detector.get_candidates()))
        # Build obstacles msgs containing all obstacles
        obstacles_msg = ros_utils.ROS_Utils.build_obstacles_msg(image_msg.header, list_of_obstacle_msgs)
        # Publish obstacles
        self.pub_obstacle.publish(obstacles_msg)

        # Get goalpost msgs and add them to the detected goal parts list
        goal_parts = ros_utils.ROS_Utils.build_goalpost_msgs(self.goalpost_detector.get_candidates())
        # Create goalparts msg
        goal_parts_msg = ros_utils.ROS_Utils.build_goal_parts_msg(image_msg.header, goal_parts)
        # Build goal message out of goal parts
        goal_msg = ros_utils.ROS_Utils.build_goal_msg(goal_parts_msg)
        # Check if there is a goal
        if goal_msg:
            # If we have a goal, lets publish it
            self.pub_goal.publish(goal_msg)

        # Build a LineSegmentInImage message for each linepoint
        line_points = self.line_detector.get_linepoints()
        # Create line segments
        line_segments = ros_utils.ROS_Utils.convert_line_points_to_line_segment_msgs(line_points)
        # Create line msg
        line_msg = ros_utils.ROS_Utils.build_line_information_in_image_msg(image_msg.header, line_segments)
        # Publish lines
        self.pub_lines.publish(line_msg)

        # Get field boundary msg
        convex_field_boundary = self.field_boundary_detector.get_convex_field_boundary_points()
        # Build ros message
        convex_field_boundary_msg = ros_utils.ROS_Utils.build_field_boundary_msg(image_msg.header, convex_field_boundary)
        # Publish field boundary
        self.pub_convex_field_boundary.publish(convex_field_boundary_msg)

        if self.config['vision_ball_detector'] == 'fcnn':
            if self.ball_fcnn_publish_output:
                self.pub_ball_fcnn.publish(self.ball_detector.get_cropped_msg())

            # Publish whole fcnn output
            if self.publish_fcnn_debug_image:
                self.pub_debug_fcnn_image.publish(self.ball_detector.get_debug_image())

        # Check if we should draw debug image
        if self.publish_debug_image:
            # Draw debug image
            debug_image = self._get_debug_image(image)
            # publish debug image
            self.pub_debug_image.publish(self.bridge.cv2_to_imgmsg(debug_image, 'bgr8'))

    @staticmethod
    def _distribute_images(image, internal_image_subscribers):
        """
        Set the image for each detector
        :param image: the current image
        """
        # Iterate over subscribers
        for vision_object in internal_image_subscribers:
            # Send image
            vision_object.set_image(image)

    def _get_debug_image(self, image):
        """
        Draws a debug image
        :param image: untouched image
        :return: image with debug annotations
        """
        # Define the debug image
        debug_image_description = [
            {
                # Unknown obstacles
                'type': "obstacle",
                'color': (0, 0, 0),
                'thickness': 3,
                'data': self.unknown_obstacle_detector.get_candidates(),
            },
            {
                # Red obstacles
                'type': "obstacle",
                'color': (0, 0, 255),
                'thickness': 3,
                'data': self.red_obstacle_detector.get_candidates(),
            },
            {
                # Blue obstacles
                'type': "obstacle",
                'color': (255, 0, 0),
                'thickness': 3,
                'data': self.blue_obstacle_detector.get_candidates(),
            },
            {
                # Goal posts
                'type': "obstacle",
                'color': (255, 255, 255),
                'thickness': 3,
                'data': self.goalpost_detector.get_candidates(),
            },
            {
                # Field boundary
                'type': "field_boundary",
                'color': (0, 0, 255),
                'thickness': 1,
                'data': self.field_boundary_detector.get_field_boundary_points(),
            },
            {
                # Convex field boundary
                'type': "field_boundary",
                'color': (0, 255, 255),
                'thickness': 1,
                'data': self.field_boundary_detector.get_convex_field_boundary_points(),
            },
            {
                # All ball candidates
                'type': "ball",
                'color': (0, 0, 255),
                'thickness': 1,
                'data': self.ball_detector.get_candidates(),
            },
            {
                # Possible ball candidates
                'type': "ball",
                'color': (0, 255, 255),
                'thickness': 1,
                'data': self.field_boundary_detector.balls_under_field_boundary(
                            self.ball_detector.get_candidates(),
                            self._ball_candidate_y_offset)
            },
            {
                # Top ball candidate
                'type': "ball",
                'color': (0, 255, 0),
                'thickness': 1,
                'data': [self.top_ball_candidate]
            },
            {
                # Line points
                'type': "line_point",
                'color': (0, 0, 255),
                'thickness': -1,
                'data': self.line_detector.get_linepoints(),
            },
        ]

        # Submit image to the debug image drawer
        self.debug_image_drawer.set_image(image)

        # Draw and return image from the debug image drawer
        return self.debug_image_drawer.draw(debug_image_description)

    def _conventional_precalculation(self):
        """
        Kicks of the conventional calculations
        """
        self.obstacle_detector.compute_all_obstacles()
        self.line_detector.compute_linepoints()

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
            ros_utils.ROS_Utils.speak("Hey!   Remove my camera cap!", self.speak_publisher)


if __name__ == '__main__':
    Vision()

#! /usr/bin/env python3

import os
import cv2
import yaml
import rospy
import rospkg
import threading
from profilehooks import profile, timecall
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.encoding import Config as DynamicReconfigureConfig
from sensor_msgs.msg import Image, JointState
from humanoid_league_msgs.msg import BallInImage, BallsInImage, LineInformationInImage, \
    LineSegmentInImage, ObstaclesInImage, ObstacleInImage, ImageWithRegionOfInterest, GoalPartsInImage, PostInImage, \
    GoalInImage, Speak
from bitbots_vision.vision_modules import lines, field_boundary, color, debug, live_classifier, \
    classifier, ball, fcnn_handler, live_fcnn_03, dummy_ballfinder, obstacle, evaluator, yolo_handler
from bitbots_vision.cfg import VisionConfig
from bitbots_msgs.msg import Config

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

        # the head_joint_states is used by the dynamic field_boundary detector
        self.head_joint_state = None

        # Publisher placeholder
        self.pub_balls = None
        self.pub_lines = None
        self.pub_obstacle = None
        self.pub_goal = None
        self.pub_ball_fcnn = None
        self.pub_debug_image = None
        self.pub_debug_fcnn_image = None

        # Subsciber placeholder
        self.image_sub = None
        self.head_sub = None

        self.debug_image_drawer = debug.DebugImage()  # Todo: better variable name
        if self.debug_image_drawer:
            self.runtime_evaluator = evaluator.RuntimeEvaluator(None)


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
        self._first_callback = True

        # Register VisionConfig server (dynamic reconfigure) and set callback
        srv = Server(VisionConfig, self._dynamic_reconfigure_callback)
        rospy.spin()

    def _image_callback(self, image_msg):
        # type: (Image) -> None
        """
        This method is called by the Image-message subscriber.
        Old Image-messages were dropped.

        Sometimes the queue gets to large, even when the size is limeted to 1. 
        That's, why we drop old images manually.
        """
        # drops old images and cleans up queue
        image_age = rospy.get_rostime() - image_msg.header.stamp 
        if image_age.to_sec() > 1.0:
            self.debug_printer.info('Vision: Dropped Image-message', 'image')
            return

        self.handle_image(image_msg)

    def _head_joint_state_callback(self, headjoint_msg):
        # type: (JointState) -> None
        """
        Sets a new head_joint_state for the field-boundary-module when a new msg is received
        :param headjoint_msg: the current vertical position of the head
        """
        self.field_boundary_detector.set_head_joint_state(headjoint_msg)

    def _speak(self, string, speech_publisher):
        """
        Sends a speak message and let the robot say the given string.
        :param string: Text the robot should say
        :param speech_publisher: ROS publisher for the speech message
        """
        speak_message = Speak()
        speak_message.text = string
        speech_publisher.publish(speak_message)

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

        # Check if its the first callback
        if self._first_callback:
            # Calc the mean brightness of the image to detect forgotten camera caps
            mean = cv2.mean(image)

            if sum(mean) < self._blind_threshold:
                self._speak("Hey!   Remove my camera cap!", self.speak_publisher)

        # distribute the image to the detectors
        self._distribute_images(image)

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

        # TODO: handle all ball candidates

        #"""

        # Grab ball candidates from ball detector
        ball_candidates = self.ball_detector.get_candidates()

        # Check if there are any ball candidates
        if ball_candidates:
            # Only take candidates under the convex field boundary
            balls_under_field_boundary = self.field_boundary_detector.balls_under_convex_field_boundary(ball_candidates)
            # Check if there are still candidates left
            if balls_under_field_boundary:
                # Sort candidates and take the one which has the biggest confidence
                sorted_rated_candidates = sorted(balls_under_field_boundary, key=lambda x: x.rating)
                top_ball_candidate = list([max(sorted_rated_candidates[0:1], key=lambda x: x.rating)])[0]
            else:
                top_ball_candidate = None
        else:
            top_ball_candidate = None

        self.top_ball_candidate = top_ball_candidate

        # check whether ball candidates are over rating threshold
        if top_ball_candidate and top_ball_candidate.get_rating() > self._ball_candidate_threshold:
            # create ball msg
            # TODO: publish empty msg if no top candidate as described in msg description
            balls_msg = BallsInImage()
            balls_msg.header.frame_id = image_msg.header.frame_id
            balls_msg.header.stamp = image_msg.header.stamp

            # Build the ball message which will be embedded in the balls message
            ball_msg = self._build_ball_msg(top_ball_candidate)
            balls_msg.candidates.append(ball_msg)

            # Publish balls
            self.pub_balls.publish(balls_msg)

        # Create obstacle msg
        obstacles_msg = ObstaclesInImage()

        # Add header
        obstacles_msg.header.frame_id = image_msg.header.frame_id
        obstacles_msg.header.stamp = image_msg.header.stamp

        # Add red obstacles
        obstacles_msg.obstacles.extend(self._build_obstacle_msgs(ObstacleInImage.ROBOT_MAGENTA, self.red_obstacle_detector.get_candidates()))
        # Add blue obstacles
        obstacles_msg.obstacles.extend(self._build_obstacle_msgs(ObstacleInImage.ROBOT_CYAN, self.blue_obstacle_detector.get_candidates()))
        # Add UFO's (Undefined Found Obstacles)
        obstacles_msg.obstacles.extend(self._build_obstacle_msgs(ObstacleInImage.UNDEFINED, self.unknown_obstacle_detector.get_candidates()))

        # Publish obstacles
        self.pub_obstacle.publish(obstacles_msg)

        # Areate goalparts msg
        goal_parts_msg = GoalPartsInImage()
        # Add header
        goal_parts_msg.header.frame_id = image_msg.header.frame_id
        goal_parts_msg.header.stamp = image_msg.header.stamp

        # Add detected goal parts to the message
        goal_parts_msg.posts.extend(self._build_goalpost_msgs(self.goalpost_detector.get_candidates()))

        # Build goal message out of goal parts
        goal_msg = self._build_goal_msg(goal_parts_msg)

        # Check if there is a goal
        if goal_msg:
            # If we have a goal, lets publish it
            self.pub_goal.publish(goal_msg)

        # Create line msg
        line_msg = LineInformationInImage()  # Todo: add lines
        line_msg.header.frame_id = image_msg.header.frame_id
        line_msg.header.stamp = image_msg.header.stamp

        # Build a LineSegmentInImage message for each linepoint
        for lp in self.line_detector.get_linepoints():
            # Create LineSegmentInImage message
            ls = LineSegmentInImage()
            ls.start.x = lp[0]
            ls.start.y = lp[1]
            ls.end = ls.start
            line_msg.segments.append(ls)
        # Publish lines
        self.pub_lines.publish(line_msg)

        # Publish fcnn output under the field boundary
        if self.ball_fcnn_publish_output and self.config['vision_ball_classifier'] == 'fcnn':
            self.pub_ball_fcnn.publish(self.ball_detector.get_cropped_msg())

        # Publish whole fcnn output
        if self.publish_fcnn_debug_image and self.config['vision_ball_classifier'] == 'fcnn':
            self.pub_debug_fcnn_image.publish(self.ball_detector.get_debug_image())

        # Check if we should draw debug image
        if self.publish_debug_image:
            #Draw debug image
            debug_image = self._get_debug_image(image)
            # publish debug image
            self.pub_debug_image.publish(self.bridge.cv2_to_imgmsg(debug_image, 'bgr8'))
        
        # Now this is not the first callback anymore
        self._first_callback = False

    def _distribute_images(self, image):
        """
        Set the image for each detector
        :param image: the current image
        """
        # TODO make Subscriber
        self.field_boundary_detector.set_image(image)
        self.obstacle_detector.set_image(image)
        self.red_obstacle_detector.set_image(image)
        self.blue_obstacle_detector.set_image(image)
        self.goalpost_detector.set_image(image)
        self.line_detector.set_image(image)
        self.ball_detector.set_image(image)
        self.runtime_evaluator.set_image()

    def _build_goal_msg(self, goal_parts_msg):
        """
        Builds a goal message with a right and left post. If there is only one post in the image, the right and left post are the same. 
        This should be reworked! The vision should only publish posts and e.g. the worldmodel builds a goal out of this context. 
        :param top_ball_candidate: best rated ball candidate
        :return: ball msg
        """
        # Make new goal message
        goal_msg = GoalInImage()
        # Add header of the goal parts
        goal_msg.header = goal_parts_msg.header
        # Create goal posts at unrealistic high/low values
        left_post = PostInImage()
        left_post.foot_point.x = 9999999999
        left_post.confidence = 1.0
        right_post = PostInImage()
        right_post.foot_point.x = -9999999999
        right_post.confidence = 1.0

        # Set our posts
        for post in goal_parts_msg.posts:
            # Decide if its a left post
            if post.foot_point.x < left_post.foot_point.x:
                left_post = post
                left_post.confidence = post.confidence
            # Decide if its a right post
            if post.foot_point.x > right_post.foot_point.x:
                right_post = post
                right_post.confidence = post.confidence

        # Set posts in message
        goal_msg.left_post = left_post
        goal_msg.right_post = right_post
        goal_msg.confidence = 1.0
        # Return message if there are any posts
        if goal_parts_msg.posts:
            return goal_msg


    def _build_ball_msg(self, top_ball_candidate):
        """
        Builds a ball message
        :param top_ball_candidate: best rated ball candidate
        :return: ball msg
        """
        # Create a empty ball message
        ball_msg = BallInImage()
        ball_msg.center.x = top_ball_candidate.get_center_x()
        ball_msg.center.y = top_ball_candidate.get_center_y()
        ball_msg.diameter = top_ball_candidate.get_diameter()
        ball_msg.confidence = top_ball_candidate.get_rating()
        self.debug_printer.info('found a ball! \o/', 'ball')
        return ball_msg

    def _build_goalpost_msgs(self, goalposts):
        """
        Builds a list of goalpost messages
        :param goalposts: goalpost candidates 
        :return: list of goalposts msgs
        """
        # Create an empty list of goalposts
        message_list = []
        # Iterate over all goalpost candidates 
        for goalpost in goalposts:
            # Create a empty post message
            post_msg = PostInImage()
            post_msg.width = goalpost.get_width()
            post_msg.confidence = goalpost.get_rating()
            post_msg.foot_point.x = goalpost.get_center_x()
            post_msg.foot_point.y = goalpost.get_lower_right_y()
            post_msg.top_point = post_msg.foot_point
            message_list.append(post_msg)
        return message_list

    def _build_obstacle_msgs(self, obstacle_color, detections):
        """
        Builds a list of obstacles for a certain color
        :param obstacle_color: color of the obstacles
        :param detections: obstacle candidates 
        :return: list of obstacle msgs
        """
        message_list = []
        for obstacle in detections:
            obstacle_msg = ObstacleInImage()
            obstacle_msg.color = obstacle_color
            obstacle_msg.top_left.x = obstacle.get_upper_left_x()
            obstacle_msg.top_left.y = obstacle.get_upper_left_y()
            obstacle_msg.height = int(obstacle.get_height())
            obstacle_msg.width = int(obstacle.get_width())
            obstacle_msg.confidence = obstacle.get_rating()
            obstacle_msg.playerNumber = 42
            message_list.append(obstacle_msg)
        return message_list

    def _get_debug_image(self, image):
        """
        Draws a debug image
        :param image: untouched image
        :return: image with debug annotations
        """
        # Submit image to the debug image drawer
        self.debug_image_drawer.set_image(image)
        # Draw unknown obstacles
        self.debug_image_drawer.draw_obstacle_candidates(
            self.unknown_obstacle_detector.get_candidates(),
            (0, 0, 0),
            thickness=3
        )
        # Draw red obstacles
        self.debug_image_drawer.draw_obstacle_candidates(
            self.red_obstacle_detector.get_candidates(),
            (0, 0, 255),
            thickness=3
        )
        # Draw blue obstacles
        self.debug_image_drawer.draw_obstacle_candidates(
            self.blue_obstacle_detector.get_candidates(),
            (255, 0, 0),
            thickness=3
        )            
        # Draw goal post obstacles
        self.debug_image_drawer.draw_obstacle_candidates(
            self.goalpost_detector.get_candidates(),
            (255, 255, 255),
            thickness=3
        )
        # Draw field boundary
        self.debug_image_drawer.draw_field_boundary(
            self.field_boundary_detector.get_field_boundary_points(),
            (0, 0, 255))
        # Draw convex field boundary
        self.debug_image_drawer.draw_field_boundary(
            self.field_boundary_detector.get_convex_field_boundary_points(),
            (0, 255, 255))
        # Draw all ball candidates
        self.debug_image_drawer.draw_ball_candidates(
            self.ball_detector.get_candidates(),
            (0, 0, 255))
        # Draw possible ball candidates
        self.debug_image_drawer.draw_ball_candidates(
            self.field_boundary_detector.balls_under_field_boundary(
                self.ball_detector.get_candidates(),
                self._ball_candidate_y_offset),
            (0, 255, 255))
        # Draw top ball candidate
        self.debug_image_drawer.draw_ball_candidates([self.top_ball_candidate],
                                                    (0, 255, 0))
        # Draw linepoints
        self.debug_image_drawer.draw_points(
            self.line_detector.get_linepoints(),
            (0, 0, 255))

        # Return image returned from the debug image drawer
        return self.debug_image_drawer.get_image()

    def _conventional_precalculation(self):
        """
        Kicks of the conventional calculations
        """
        self.obstacle_detector.compute_all_obstacles()
        self.line_detector.compute_linepoints()

    def _dynamic_reconfigure_callback(self, config, level):
        """
        Callback for the dynamic reconfigure configuration. This callback also gets calles for the inertial configuration.
        :param config: New config
        :param level: No idea what this is for. I google this if we are landed #TODO
        """
        self.debug_printer = debug.DebugPrinter(
            debug_classes=debug.DebugPrinter.generate_debug_class_list_from_string(
                config['vision_debug_printer_classes']))

        # Inits runtime evaluator 
        self.runtime_evaluator = evaluator.RuntimeEvaluator(self.debug_printer)

        # Set some thresholds
        # Brightness threshold which determins if the camera cap is on the camera. 
        self._blind_threshold = config['vision_blind_threshold']
        # Threshold for ball candidates
        self._ball_candidate_threshold = config['vision_ball_candidate_rating_threshold']
        # Maximum offset for balls over the convex field boundary
        self._ball_candidate_y_offset = config['vision_ball_candidate_field_boundary_y_offset']

        # Should the debug image be published?
        self.publish_debug_image = config['vision_publish_debug_image']
        if self.publish_debug_image:
            rospy.logwarn('Debug images are enabled')
        else:
            rospy.loginfo('Debug images are disabled')

        # Should the fcnn output (only under the field boundary) be published?
        self.ball_fcnn_publish_output = config['ball_fcnn_publish_output']
        if self.ball_fcnn_publish_output:
            rospy.logwarn('ball FCNN output publishing is enabled')
        else:
            rospy.logwarn('ball FCNN output publishing is disabled')

        # Should the whole fcnn output be published?
        self.publish_fcnn_debug_image = config['ball_fcnn_publish_debug_img']


        # Print if the vision uses the sim color or not (only prints when it changes or its the fist callback)
        if 'vision_use_sim_color' not in self.config or \
            config['vision_use_sim_color'] != self.config['vision_use_sim_color']:
            if config['vision_use_sim_color']:
                rospy.logwarn('Loaded color space for SIMULATOR.')
            else:
                rospy.loginfo('Loaded color space for REAL WORLD.')

        # Set the white color detector
        self.white_color_detector = color.HsvSpaceColorDetector(
            self.debug_printer,
            [config['white_color_detector_lower_values_h'], config['white_color_detector_lower_values_s'],
             config['white_color_detector_lower_values_v']],
            [config['white_color_detector_upper_values_h'], config['white_color_detector_upper_values_s'],
             config['white_color_detector_upper_values_v']])

        # Set the red color detector
        self.red_color_detector = color.HsvSpaceColorDetector(
            self.debug_printer,
            [config['red_color_detector_lower_values_h'], config['red_color_detector_lower_values_s'],
             config['red_color_detector_lower_values_v']],
            [config['red_color_detector_upper_values_h'], config['red_color_detector_upper_values_s'],
             config['red_color_detector_upper_values_v']])

        # Set the blue color detector
        self.blue_color_detector = color.HsvSpaceColorDetector(
            self.debug_printer,
            [config['blue_color_detector_lower_values_h'], config['blue_color_detector_lower_values_s'],
             config['blue_color_detector_lower_values_v']],
            [config['blue_color_detector_upper_values_h'], config['blue_color_detector_upper_values_s'],
             config['blue_color_detector_upper_values_v']])

        # Check if the dynamic color space field color detector or the static field color detector should be used
        if config['dynamic_color_space_active']:
            # Set dynamic color space field color detector
            self.field_color_detector = color.DynamicPixelListColorDetector(
                self.debug_printer,
                self.package_path,
                config,
                primary_detector=True)
        else:
            # Set the static field color detector
            self.field_color_detector = color.PixelListColorDetector(
                self.debug_printer,
                self.package_path,
                config)

        # Set the field boundary detector
        self.field_boundary_detector = field_boundary.FieldBoundaryDetector(
            self.field_color_detector,
            config,
            self.debug_printer,
            self.runtime_evaluator)

        # Set the line detector
        self.line_detector = lines.LineDetector(
            self.white_color_detector,
            self.field_color_detector,
            self.field_boundary_detector,
            config,
            self.debug_printer)

        # Set the obstacle detector
        self.obstacle_detector = obstacle.ObstacleDetector(
            self.red_color_detector,
            self.blue_color_detector,
            self.white_color_detector,
            self.field_boundary_detector,
            self.runtime_evaluator,
            config,
            self.debug_printer
        )

        # If we don't use YOLO set the conventional goalpost detector.
        if not config['vision_ball_classifier'] in ['yolo_opencv', 'yolo_darknet']:
            self.goalpost_detector = obstacle.WhiteObstacleDetector(self.obstacle_detector)
        # Set the other obstacle detectors
        self.red_obstacle_detector = obstacle.RedObstacleDetector(self.obstacle_detector)
        self.blue_obstacle_detector = obstacle.BlueObstacleDetector(self.obstacle_detector)
        self.unknown_obstacle_detector = obstacle.UnknownObstacleDetector(self.obstacle_detector)

        # If dummy ball detection is activated, set the dummy ballfinder as ball detector
        if config['vision_ball_classifier'] == 'dummy':
            self.ball_detector = dummy_ballfinder.DummyClassifier(None, None, self.debug_printer)

        # set up ball config for fcnn
        # these config params have domain-specific names which could be problematic for fcnn handlers handling e.g. goal candidates
        # this enables 2 fcnns with different configs.
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

        # Check if the fcnn ball detector is activated 
        if config['vision_ball_classifier'] == 'fcnn':
            # Check if its the first callback, the fcnn is newly activated or the model has changed
            if 'neural_network_model_path' not in self.config or self.config['neural_network_model_path'] != config['neural_network_model_path'] or self.config['vision_ball_classifier'] != config['vision_ball_classifier']:
                # Build absolute model path
                ball_fcnn_path = os.path.join(self.package_path, 'models', config['neural_network_model_path'])
                # Check if it exists
                if not os.path.exists(os.path.join(ball_fcnn_path, "model_final.index")):
                    rospy.logerr('AAAAHHHH! The specified fcnn model file doesn\'t exist! Maybe its a YOLO model? Look twice.')
                else:
                    self.ball_fcnn = live_fcnn_03.FCNN03(ball_fcnn_path, self.debug_printer)
                    rospy.loginfo("FCNN vision is running now")
            self.ball_detector = fcnn_handler.FcnnHandler(
                self.ball_fcnn,
                self.field_boundary_detector,
                self.ball_fcnn_config,
                self.debug_printer)

        # Check if the yolo ball/goalpost detector is activated. No matter which implementation is used.
        if config['vision_ball_classifier'] in ['yolo_opencv', 'yolo_darknet']:
            if 'neural_network_model_path' not in self.config or self.config['neural_network_model_path'] != config['neural_network_model_path'] or self.config['vision_ball_classifier'] != config['vision_ball_classifier']:
                # Build absolute model path
                yolo_model_path = os.path.join(self.package_path, 'models', config['neural_network_model_path'])
                # Check if it exists
                if not os.path.exists(os.path.join(yolo_model_path, "yolo_weights.weights")):
                    rospy.logerr('AAAAHHHH! The specified yolo model file doesn\'t exist! Maybe its an fcnn model?')
                else:
                    # Decide which yolo implementation should be used
                    if config['vision_ball_classifier'] == 'yolo_opencv':
                        # Load OpenCV implementation (uses OpenCL)
                        yolo = yolo_handler.YoloHandlerOpenCV(config, yolo_model_path)
                    else:
                        # Load Darknet implementation (uses CUDA)
                        yolo = yolo_handler.YoloHandlerDarknet(config, yolo_model_path)
                    # Set both ball and goalpost detector
                    self.ball_detector = yolo_handler.YoloBallDetector(yolo)
                    self.goalpost_detector = yolo_handler.YoloGoalpostDetector(yolo)
                    rospy.loginfo(config['vision_ball_classifier'] + " vision is running now")

            
        # Now register all publishers
        # TODO: topic: ball_in_... BUT MSG TYPE: balls_in_img... CHANGE TOPIC TYPE!

        self.pub_balls = self._create_or_update_publisher(self.config, config, 'ROS_ball_msg_topic', BallsInImage, self.pub_balls)

        self.pub_lines = self._create_or_update_publisher(self.config, config, 'ROS_line_msg_topic', LineInformationInImage, self.pub_lines, queue_size=5)

        self.pub_obstacle = self._create_or_update_publisher(self.config, config, 'ROS_obstacle_msg_topic', ObstaclesInImage, self.pub_obstacle, queue_size=3)

        self.pub_goal = self._create_or_update_publisher(self.config, config, 'ROS_goal_msg_topic', GoalInImage, self.pub_goal, queue_size=3)

        self.pub_ball_fcnn = self._create_or_update_publisher(self.config, config, 'ROS_fcnn_img_msg_topic', ImageWithRegionOfInterest, self.pub_ball_fcnn)

        self.pub_debug_image = self._create_or_update_publisher(self.config, config, 'ROS_debug_image_msg_topic', Image, self.pub_debug_image)

        self.pub_debug_fcnn_image = self._create_or_update_publisher(self.config, config, 'ROS_debug_fcnn_image_msg_topic', Image, self.pub_debug_fcnn_image)

        # subscribers

        self.image_sub = self._create_or_update_subscriber(self.config, config, 'ROS_img_msg_topic', Image, self.image_sub, self._image_callback, queue_size=config['ROS_img_queue_size'], buff_size=60000000)

        # TODO replace with transform from basefootprint to camera_optical_frame
        # subscriber for the vertical position of the head, used by the dynamic field-boundary-detector
        self.head_sub = self._create_or_update_subscriber(self.config, config, 'ROS_head_joint_msg_topic', JointState, self.head_sub, self._head_joint_state_callback, queue_size=config['ROS_head_joint_state_queue_size'])

        # Publish Config-message (mainly for the dynamic color space node)
        self._publish_vision_config(config)

        # The old config gets replaced with the new config
        self.config = config
        return config

    def _create_or_update_publisher(self, old_config, new_config, topic_key, message_type, publisher_object, queue_size=1):
        """
        Creates or updates an publisher
        :param old_config: Previous config entries
        :param new_config: Current config entries
        :param topic_key: The config key, where the topic name is stored
        :param message_type: The ROS message type of the topic we want to publish
        :param publisher_object: The python object, that represents the publisher
        :param queue_size: The ROS message queue size
        :return: adjusted publisher object
        """
        # Check if its the first call or the topic changed
        if topic_key not in old_config or old_config[topic_key] != new_config[topic_key]:
            # Check if an publisher exists and unregister him
            if publisher_object is not None:
                publisher_object.unregister()
            # Create the new publisher
            publisher_object = rospy.Publisher(
                new_config[topic_key],
                message_type,
                queue_size=queue_size)
            rospy.loginfo("Registered new publisher to " + str(new_config[topic_key]))
        return publisher_object

    def _create_or_update_subscriber(self, old_config, new_config, topic_key, message_type, subscriber_object, callback, queue_size=1, buff_size=65536):
        """
        Creates or updates an subscriber
        :param old_config: Previous config entries
        :param new_config: Current config entries
        :param topic_key: The config key, where the topic name is stored
        :param message_type: The ROS message type of the topic we want to subscribe
        :param subscriber_object: The python object, that represents the subscriber
        :param callback: The subscriber callback function
        :param queue_size: The ROS message queue size
        :param buff_size: The ROS message buffer size
        :return: adjusted subscriber object
        """
        # Check if its the first call or the topic changed
        if topic_key not in old_config or old_config[topic_key] != new_config[topic_key]:
            # Check if an subsciber exists and unregister him
            if subscriber_object is not None:
                subscriber_object.unregister()
            # Create the new subscriber
            subscriber_object = rospy.Subscriber(
                new_config[topic_key],
                message_type,
                callback,
                queue_size=queue_size,
                tcp_nodelay=True,
                buff_size=buff_size)
            rospy.loginfo("Registered new subscriber at " + str(new_config[topic_key]))
        return subscriber_object

    def _publish_vision_config(self, config):
        """
        Publishes the given config. 
        :param config: A vision config
        """
        # Clean config dict to avoid not dumpable types
        config_cleaned = {}
        # Iterate over all config keys and values
        for key, value in config.items():
            # Check if the value is dumpable
            if type(value) != DynamicReconfigureConfig:
                config_cleaned[key] = value
        # Create new config message
        msg = Config()
        # The message contains a string. So the config gets serialized and send as string
        msg.data = yaml.dump(config_cleaned)
        # Publish config
        self.pub_config.publish(msg)
            
            

if __name__ == '__main__':
    Vision()

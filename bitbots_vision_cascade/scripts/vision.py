#! /usr/bin/env python2


from bitbots_vision_common.vision_modules import lines, horizon, color, debug, live_classifier, classifier, ball, lines2
from humanoid_league_msgs.msg import BallInImage, BallsInImage, LineInformationInImage, LineSegmentInImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import rospkg
import cv2
import os
from dynamic_reconfigure.server import Server
from bitbots_vision_cascade.cfg import VisionConfig


class Vision:

    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('bitbots_vision_cascade')

        rospy.init_node('bitbots_vision_cascade')
        rospy.loginfo('Initializing cascade vision...')
        self.config = {}
        # register config callback and set config
        srv = Server(VisionConfig, self._dynamic_reconfigure_callback)

        self.bridge = CvBridge()

        rospy.spin()

    def _image_callback(self, img):
        self.handle_image(img)

    def handle_image(self, image_msg):
        # converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # setup detectors
        horizon_detector = horizon.HorizonDetector(image,
                                                   self.field_color_detector,
                                                   self.horizon_config)
        ball_finder = ball.BallFinder(image, self.cascade, self.ball_config)
        ball_classifier = classifier. \
            Classifier(image,
                       self.ball_classifier,
                       horizon_detector.
                       balls_under_horizon(
                           ball_finder.get_ball_candidates(),
                           self._ball_candidate_y_offset))

        top_ball_candidate = ball_classifier.get_top_candidate()

        line_detector = lines.LineDetector(image,
                                           [top_ball_candidate] if top_ball_candidate else list(),
                                           self.white_color_detector,
                                           horizon_detector,
                                           self.lines_config)

        # do debug stuff
        if self.debug:
            debug_image_dings = debug.DebugImage(image)
            debug_image_dings.draw_horizon(
                horizon_detector.get_horizon_points(),
                (0, 0, 255))
            debug_image_dings.draw_ball_candidates(
                ball_classifier.get_candidates(),
                (0, 0, 255))
            debug_image_dings.draw_ball_candidates(
                horizon_detector.balls_under_horizon(
                    ball_classifier.get_candidates(),
                    self._ball_candidate_y_offset),
                (0, 255, 255))

        # create ball msg
        if top_ball_candidate and top_ball_candidate.rating > self._ball_candidate_threshold:
            if self.debug:
                debug_image_dings.draw_ball_candidates([top_ball_candidate],
                                                       (0, 255, 0))
            balls_msg = BallsInImage()
            balls_msg.header.frame_id = image_msg.header.frame_id
            balls_msg.header.stamp = image_msg.header.stamp

            ball_msg = BallInImage()
            ball_msg.center.x = top_ball_candidate.get_center_x()
            ball_msg.center.y = top_ball_candidate.get_center_y()
            ball_msg.diameter = top_ball_candidate.get_diameter()
            ball_msg.confidence = 1

            balls_msg.candidates.append(ball_msg)
            rospy.loginfo('found a ball! \o/')
            self.pub_balls.publish(balls_msg)

        # create line msg
        line_msg = LineInformationInImage()  # Todo: add lines
        line_msg.header.frame_id = image_msg.header.frame_id
        line_msg.header.stamp = image_msg.header.stamp
        for lp in line_detector.get_linepoints():
            ls = LineSegmentInImage()
            ls.start.x = lp[0]
            ls.start.y = lp[1]
            ls.end = ls.start
            line_msg.segments.append(ls)
        self.pub_lines.publish(line_msg)
        if self.debug:
            # draw linepoints in black
            debug_image_dings.draw_points(
                line_detector.get_linepoints(),
                (0, 0, 0))
            # debug_image_dings.draw_line_segments(line_detector.get_linesegments(), (255, 0, 0))
            debug_image_dings.imshow()

    def handle_image_no_balls(self, image_msg):
        # converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # setup detectors
        horizon_detector = horizon.HorizonDetector(image,
                                                   self.field_color_detector,
                                                   self.horizon_config)

        line_detector = lines.LineDetector(image,
                                           list(),
                                           self.white_color_detector,
                                           horizon_detector,
                                           self.lines_config)

        # create line msg
        line_msg = LineInformationInImage()  # Todo: add lines
        line_msg.header.frame_id = image_msg.header.frame_id
        line_msg.header.stamp = image_msg.header.stamp
        for lp in line_detector.get_linepoints():
            ls = LineSegmentInImage()
            ls.start.x = lp[0]
            ls.start.y = lp[1]
            ls.end = ls.start
            line_msg.segments.append(ls)
        self.pub_lines.publish(line_msg)
        if self.debug:
            # draw linepoints in black
            debug_image_dings = debug.DebugImage(image)
            debug_image_dings.draw_horizon(
                horizon_detector.get_horizon_points(),
                (0, 0, 255))
            debug_image_dings.draw_points(
                line_detector.get_linepoints(),
                (0, 0, 0))
            debug_image_dings.imshow()

    def _dynamic_reconfigure_callback(self, config, level):
        self._ball_candidate_threshold = config["vision_ball_candidate_rating_threshold"]
        self._ball_candidate_y_offset = config["vision_ball_candidate_horizon_y_offset"]

        self.debug = config["vision_debug"]
        if self.debug:
            rospy.logwarn("Debug windows are enabled")
        else:
            rospy.loginfo("Debug windows are disabled")

        self.no_balls = config["vision_no_balls"]

        self.cascade_path = self.package_path + config["cascade_classifier_path"]
        if "cascade_classifier_path" not in self.config or \
                self.config["cascade_classifier_path"] != config["cascade_classifier_path"]:
            if os.path.exists(self.cascade_path):
                self.cascade = cv2.CascadeClassifier(self.cascade_path)
            else:
                rospy.logerr(
                    'AAAAHHHH! The specified cascade config file doesn\'t exist!')
        self.ball_classifier = live_classifier \
            .LiveClassifier(self.package_path + config["classifier_model_path"])

        # set up ball config
        self.ball_config = {
            'classify_threshold': config["ball_finder_classify_threshold"],
            'scale_factor': config["ball_finder_scale_factor"],
            'min_neighbors': config["ball_finder_min_neighbors"],
            'min_size': config["ball_finder_min_size"],
        }

        # color config
        self.white_color_detector = color.HsvSpaceColorDetector(
            [config["white_color_detector_lower_values_h"], config["white_color_detector_lower_values_s"], config["white_color_detector_lower_values_v"]],
            [config["white_color_detector_upper_values_h"], config["white_color_detector_upper_values_s"], config["white_color_detector_upper_values_v"]])

        self.field_color_detector = color.PixelListColorDetector(
            self.package_path +
            config["field_color_detector_path"])

        # set up horizon config
        self.horizon_config = {
            'x_steps': config["horizon_finder_horizontal_steps"],
            'y_steps': config["horizon_finder_vertical_steps"],
            'precise_pixel': config["horizon_finder_precision_pix"],
            'min_precise_pixel': config["horizon_finder_min_precision_pix"],
        }

        # set up lines config
        self.lines_config = {
            'horizon_offset': config["line_detector_horizon_offset"],
            'linepoints_range':  config["line_detector_linepoints_range"],
            'blur_kernel_size': config["line_detector_blur_kernel_size"],
        }


        # subscribers
        if "ROS_img_msg_topic" not in self.config or \
                self.config["ROS_img_msg_topic"] != config["ROS_img_msg_topic"]:
            if hasattr(self, 'image_sub'):
                self.image_sub.unregister()
            if config["vision_no_balls"]:
                callback = self.handle_image_no_balls
            else:
                callback = self._image_callback
            self.image_sub = rospy.Subscriber(config["ROS_img_msg_topic"],
                                              Image,
                                              callback,
                                              queue_size=config["ROS_img_queue_size"],
                                              buff_size=60000000)
            # https://github.com/ros/ros_comm/issues/536

        # publishers
        if "ROS_ball_msg_topic" not in self.config or \
                self.config["ROS_ball_msg_topic"] != config["ROS_ball_msg_topic"]:
            if hasattr(self, 'pub_balls'):
                self.pub_balls.unregister()
            self.pub_balls = rospy.Publisher(
                config["ROS_ball_msg_topic"],
                BallsInImage,
                queue_size=1)

        if "ROS_line_msg_topic" not in self.config or \
                self.config["ROS_line_msg_topic"] != config["ROS_line_msg_topic"]:
            if hasattr(self, 'pub_lines'):
                    self.pub_lines.unregister()
            self.pub_lines = rospy.Publisher(
                config["ROS_line_msg_topic"],
                LineInformationInImage,
                queue_size=5)
        self.config = config

        return config



if __name__ == "__main__":
    Vision()




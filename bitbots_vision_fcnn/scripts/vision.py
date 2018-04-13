#! /usr/bin/env python2


from bitbots_vision_common.vision_modules import lines, horizon, color, debug, fcnn_handler, live_fcnn_03, ball
from humanoid_league_msgs.msg import BallInImage, BallsInImage, LineInformationInImage, LineSegmentInImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import rospkg
import cv2
import os


class Vision:

    def __init__(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('bitbots_vision_fcnn')

        self._ball_candidate_threshold = rospy.get_param(
            'visionparams/vision/ball_candidate_rating_threshold')
        self._ball_candidate_y_offset = rospy.get_param(
            'visionparams/vision/ball_candidate_horizon_y_offset')

        self.debug = rospy.get_param('visionparams/vision/debug')

        ball_fcnn_path = package_path + \
            rospy.get_param('visionparams/ball_fcnn/model_path')
        if not os.path.exists(ball_fcnn_path):
            rospy.logerr(
                'AAAAHHHH! The specified fcnn model file doesn\'t exist!')
        self.ball_fcnn = live_fcnn_03.FCNN03(ball_fcnn_path)

        # set up ball fcnn config
        self.ball_fcnn_config = {
            'debug': rospy.get_param('visionparams/ball_fcnn/debug')
                     and self.debug,
            'threshold': rospy.get_param('visionparams/ball_fcnn/threshold'),
            'expand_stepsize': rospy.get_param('visionparams/ball_fcnn/expand_stepsize'),
            'pointcloud_stepsize': rospy.get_param('visionparams/ball_fcnn/pointcloud_stepsize'),
            'shuffle_candidate_list': rospy.get_param('visionparams/ball_fcnn/shuffle_candidate_list'),
            'min_ball_diameter': rospy.get_param('visionparams/ball_fcnn/min_ball_diameter'),
            'max_ball_diameter': rospy.get_param('visionparams/ball_fcnn/max_ball_diameter'),
        }

        # color config

        self.white_color_detector = color.HsvSpaceColorDetector(
            rospy.get_param('visionparams/white_color_detector/lower_values'),
            rospy.get_param('visionparams/white_color_detector/upper_values'))

        self.field_color_detector = color.PixelListColorDetector(
            package_path +
            rospy.get_param('visionparams/field_color_detector/path'))


        # set up horizon config
        self.horizon_config = {
            'x_steps': rospy.get_param(
                'visionparams/horizon_finder/horizontal_steps'),
            'y_steps': rospy.get_param(
                'visionparams/horizon_finder/vertical_steps'),
            'precise_pixel': rospy.get_param(
                'visionparams/horizon_finder/precision_pix'),
            'min_precise_pixel': rospy.get_param(
                'visionparams/horizon_finder/min_precision_pix'),
        }

        # set up lines config
        self.lines_config = {
            'horizon_offset': rospy.get_param(
                'visionparams/line_detector/horizon_offset'),
            'linepoints_range': rospy.get_param(
                'visionparams/line_detector/linepoints_range'),
            'blur_kernel_size': rospy.get_param(
                'visionparams/line_detector/blur_kernel_size'),
        }
        # ROS-Stuff:

        rospy.init_node('bitbots_vision_fcnn')
        # publisher:
        self.pub_balls = rospy.Publisher(
            rospy.get_param('visionparams/ROS/ball_msg_topic'),
            BallsInImage,
            queue_size=1)
        self.pub_lines = rospy.Publisher(
            rospy.get_param('visionparams/ROS/line_msg_topic'),
            LineInformationInImage,
            queue_size=5)

        # subscriber:
        self.bridge = CvBridge()
        rospy.Subscriber(rospy.get_param('visionparams/ROS/img_msg_topic'),
                         Image,
                         self._image_callback,
                         queue_size=rospy.get_param(
                             'visionparams/ROS/img_queue_size'),
                         buff_size=60000000)
        # https://github.com/ros/ros_comm/issues/536
        if self.debug:
            rospy.logwarn("Debug windows are enabled")
        else:
            rospy.loginfo("Debug windows are disabled")

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
        ball_fcnn_handler = fcnn_handler.FcnnHandler(image, self.ball_fcnn, self.ball_fcnn_config)
        # ball_finder = ball.BallFinder(image, self.cascade, self.ball_config)
        # ball_classifier = classifier.\
        #     Classifier(image,
        #                self.ball_classifier,
        #                horizon_detector.
        #                candidates_under_horizon(
        #                    ball_finder.get_candidates(),
        #                    self._ball_candidate_y_offset))
        top_ball_candidate = ball_fcnn_handler.get_top_candidate()
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
                    ball_fcnn_handler.get_candidates(),
                    (0, 0, 255))
            debug_image_dings.draw_ball_candidates(
                horizon_detector.balls_under_horizon(
                    ball_fcnn_handler.get_candidates(),
                    self._ball_candidate_y_offset),
                (0, 255, 255))
            debug.DebugPrinter.print_candidates_info(ball_fcnn_handler.get_candidates(), 'Ball')
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
            debug_image_dings.imshow()


if __name__ == "__main__":
    Vision()

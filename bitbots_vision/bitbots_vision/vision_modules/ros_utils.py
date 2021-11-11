import os
import re
import rclpy
import yaml
from  rclpy import logging
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32, PolygonStamped
from humanoid_league_msgs.msg import BallInImage, BallInImageArray, LineInformationInImage, LineSegmentInImage, ObstacleInImageArray, \
     ObstacleInImage, GoalPostInImageArray, GoalPostInImage, Audio, RegionOfInterestWithImage

"""
This module provides some methods needed for the ros environment,
e.g. methods to convert candidates to ROS messages or methods to modify the dynamic reconfigure objects.
"""

_cv_bridge = CvBridge()

logger = logging.get_logger('bitbots_vision')

general_parameters = []

def create_or_update_publisher(node, old_config, new_config, publisher_object, topic_key, data_class, queue_size=1):
    """
    Creates or updates a publisher

    :param node: ROS node to which the publisher is binded
    :param old_config: Previous config dict
    :param new_config: Current config dict
    :param publisher_object: The python object, that represents the publisher
    :param topic_key: The name of the topic variable in the config dict
    :param data_class: Data type class for ROS messages of the topic we want to subscribe
    :param queue_size: The ROS message queue size
    :return: adjusted publisher object
    """
    # Check if topic parameter has changed
    if config_param_change(old_config, new_config, topic_key):
        # Create the new publisher
        publisher_object = node.create_publisher(
            data_class,
            new_config[topic_key],
            queue_size)
        logger.debug("Registered new publisher to " + str(new_config[topic_key]))
    return publisher_object

def create_or_update_subscriber(node, old_config, new_config, subscriber_object, topic_key, data_class, callback, queue_size=1):
    """
    Creates or updates a subscriber

    :param node: ROS node to which the publisher is binded
    :param old_config: Previous config dict
    :param new_config: Current config dict
    :param subscriber_object: The python object, that represents the subscriber
    :param topic_key: The name of the topic variable in the config dict
    :param data_class: Data type class for ROS messages of the topic we want to subscribe
    :param callback: The subscriber callback function
    :param callback_args: Additional arguments for the callback method
    :param queue_size: The ROS message queue size
    :param buff_size: The ROS message buffer size
    :param tcp_nodelay: If True, requests tcp_nodelay from publisher
    :return: adjusted subscriber object
    """
    # Check if topic parameter has changed
    if config_param_change(old_config, new_config, topic_key):
        # Create the new subscriber
        subscriber_object = node.create_subscription(
            data_class,
            new_config[topic_key],
            callback,
            queue_size)
        logger.debug("Registered new subscriber at " + str(new_config[topic_key]))
    return subscriber_object

def build_goal_post_array_msg(header, goal_post_msgs):
    """
    Builds a GoalPostInImageArray message out of a list of GoalPostInImage messages

    :param header: ros header of the new message. Mostly the header of the image
    :param goal_post_msgs: List of goal post messages
    :return: GoalPostInImageArray message
    """
    # Create goalposts msg
    goal_posts_msg = GoalPostInImageArray()
    # Add header
    goal_posts_msg.header.frame_id = header.frame_id
    goal_posts_msg.header.stamp = header.stamp
    # Add detected goal posts to the message
    goal_posts_msg.posts = goal_post_msgs
    return goal_posts_msg

def build_goal_post_msgs(goalposts):
    """
    Builds a list of goalpost messages

    :param goalposts: goalpost candidates
    :return: List of goalpost messages
    """
    # Create an empty list of goalposts
    message_list = []
    # Iterate over all goalpost candidates
    for goalpost in goalposts:
        # Create a empty post message
        post_msg = GoalPostInImage()
        post_msg.width = float(goalpost.get_width())
        if goalpost.get_rating() is not None:
            post_msg.confidence = float(goalpost.get_rating())
        post_msg.foot_point.x = float(goalpost.get_center_x())
        post_msg.foot_point.y =float(goalpost.get_lower_right_y())
        post_msg.top_point.x = float(goalpost.get_center_x())
        post_msg.top_point.y = float(goalpost.get_upper_left_y())
        message_list.append(post_msg)
    return message_list

def build_balls_msg(header, balls):
    """
    Builds a balls message out of a list of ball messages

    :param header: ros header of the new message. Mostly the header of the image
    :param balls: A list of BallInImage messages
    :return: balls msg
    """
    # create ball msg
    balls_msg = BallInImageArray()
    # Set header
    balls_msg.header.frame_id = header.frame_id
    balls_msg.header.stamp = header.stamp
    # Add balls
    for ball in balls:
        balls_msg.candidates.append(ball)
    return balls_msg

def build_ball_msg(top_ball_candidate):
    """
    Builds a ball message

    :param top_ball_candidate: best rated ball candidate
    :return: ball msg
    """
    # Create a empty ball message
    ball_msg = BallInImage()
    ball_msg.center.x = float(top_ball_candidate.get_center_x())
    ball_msg.center.y = float(top_ball_candidate.get_center_y())
    ball_msg.diameter = float(top_ball_candidate.get_diameter())
    ball_msg.confidence = float(top_ball_candidate.get_rating())
    return ball_msg

def build_obstacle_array_msg(header, obstacles):
    """
    Builds a ObstacleInImageArray message containing a list of obstacle messages

    :param header: ros header of the new message. Mostly the header of the image
    :param obstacles: a list of obstacle messages
    :return: ObstacleInImageArray message
    """
    # Create obstacle msg
    obstacles_msg = ObstacleInImageArray()
    # Add header
    obstacles_msg.header.frame_id = header.frame_id
    obstacles_msg.header.stamp = header.stamp
    # Add obstacles
    obstacles_msg.obstacles = obstacles
    return obstacles_msg

def build_obstacle_msgs(obstacle_type, detections):
    """
    Builds a list of obstacles for a certain color

    :param obstacle_type: type of the obstacles
    :param detections: obstacle candidates
    :return: list of obstacle msgs
    """
    message_list = []
    for detected_obstacle in detections:
        obstacle_msg = ObstacleInImage()
        obstacle_msg.type = obstacle_type
        obstacle_msg.top_left.x = float(detected_obstacle.get_upper_left_x())
        obstacle_msg.top_left.y = float(detected_obstacle.get_upper_left_y())
        obstacle_msg.height = int(detected_obstacle.get_height())
        obstacle_msg.width = int(detected_obstacle.get_width())
        if detected_obstacle.get_rating() is not None:
            obstacle_msg.confidence = float(detected_obstacle.get_rating())
        else:
            obstacle_msg.confidence = 1.0
        obstacle_msg.player_number = 42
        message_list.append(obstacle_msg)
    return message_list

def build_field_boundary_polygon_msg(header, field_boundary):
    """
    Builds a PolygonStamped ROS geometry message containing the field boundary.

    :param header: ros header of the new message. Mostly the header of the image
    :param field_boundary: List of tuples containing the field boundary points.
    :return: PolygonStamped message
    """
    # Create message
    field_boundary_msg = PolygonStamped()
    # Add header
    field_boundary_msg.header = header
    # Add field boundary points
    for point in field_boundary:
        p = Point32()
        p.x = float(point[0])
        p.y = float(point[1])
        field_boundary_msg.polygon.points.append(p)

    return field_boundary_msg

def build_line_information_in_image_msg(header, line_segments):
    """
    Builds a LineInformationInImage that consists of line segments

    :param header: ros header of the new message. Mostly the header of the image
    :param line_segments: A list of LineSegmentInImage messages
    :return: Final LineInformationInImage message
    """
    # Create message
    line_msg = LineInformationInImage()
    # Set header values
    line_msg.header.frame_id = header.frame_id
    line_msg.header.stamp = header.stamp
    # Set line segments
    line_msg.segments = line_segments
    return line_msg

def build_image_msg(header, image, desired_encoding="passthrough"):
    """
    Builds a Image message

    :param header: ROS header of the new message. Mostly the header of the incoming image.
    :param image: A 2d NumPy UInt8 array
    :param desired_encoding: The Image type. E.g. 8UC[1-4], 8SC[1-4], 16UC[1-4], 16SC[1-4], 32SC[1-4], 32FC[1-4], 64FC[1-4]
    :return: The Image message
    """
    image_msg = _cv_bridge.cv2_to_imgmsg(image, desired_encoding)
    image_msg.header = header
    return image_msg

def convert_line_points_to_line_segment_msgs(line_points):
    """
    Converts a list of linepoints in the form [(x,y), ...] into a list of LineSegmentInImage messages

    :param line_points: A list of linepoints in the form [(x,y), ...]
    :return: A list of LineSegmentInImage messages
    """
    line_segments = []
    for line_point in line_points:
        # Create LineSegmentInImage message
        line_segment = LineSegmentInImage()
        line_segment.start.x = float(line_point[0])
        line_segment.start.y = float(line_point[1])
        line_segment.end = line_segment.start
        line_segments.append(line_segment)
    return line_segments

def speak(string, speech_publisher):
    """
    Sends a speak message and let the robot say the given string.

    :param string: Text the robot should say
    :param speech_publisher: ROS publisher for the speech message
    """
    speak_message = Audio()
    speak_message.text = string
    speech_publisher.publish(speak_message)

def set_general_parameters(params):
    """
    Sets params, that should trigger every `config_param_change` call.

    :params list of global params
    """
    general_parameters.extend(params)

def config_param_change(old_config, new_config, params_expressions, check_generals=True):
    # type: (dict, dict, [str]) -> bool
    """
    Checks whether some of the specified config params have changed.

    :param dict old_config: old config dict
    :param dict new_config: new config dict
    :param list of str or str params_expressions: regex discribing parameter name or list of parameter names
    :param bool check_generals: Also check for general params (Default True)
    :return bool: True if parameter has changed
    """
    # Make regex instead of list possible
    if not isinstance(params_expressions, list):
        params_expressions = [params_expressions]

    # Matching parameters
    params = []
    # Iterate over expressions
    for param in params_expressions:
        # Build regex
        regex = re.compile(param)
        # Filter parameter names by regex
        params.extend(list(filter(regex.search, list(new_config.keys()))))

    # Check if parameters matching this regex exist
    if len(params) == 0:
        raise KeyError(f"Regex '{params}' has no matches in dict.")

    # Add general params to parameters
    if check_generals:
        params.extend(general_parameters)

    # Iterate over params
    for param in params:
        # Check if param exists in new config
        if param not in new_config:
            raise KeyError(f"Parameter '{param}' is not in dict.")
        # Check if param is new or if param has changed
        elif param not in old_config or old_config[param] != new_config[param]:
            logger.debug(f"Parameter '{param}' has changed to '{new_config[param]}'")
            return True
    return False

def publish_vision_config(config, publisher):
    """
    Publishes the given config.

    :param config: A vision config
    :param publisher: The ROS publisher object
    """
    # Clean config dict to avoid not dumpable types
    config_cleaned = {}
    # Iterate over all config keys and values
    for key, value in config.items():
        # Check if the value is dumpable
        if not isinstance(value, DynamicReconfigureConfig):
            config_cleaned[key] = value
    # Create new config message
    msg = Config()
    # The message contains a string. So the config gets serialized and send as string
    msg.data = yaml.dump(config_cleaned)
    # Publish config
    publisher.publish(msg)

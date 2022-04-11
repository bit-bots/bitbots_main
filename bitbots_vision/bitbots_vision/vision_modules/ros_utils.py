import os
import re
import rclpy
import yaml
from  rclpy import logging
from cv_bridge import CvBridge
from vision_msgs.msg import BoundingBox2D, Pose2D, Point2D
from humanoid_league_msgs.msg import Audio, GameState
from soccer_vision_msgs.msg import Ball, BallArray, FieldBoundary, Goalpost, GoalpostArray, Robot, RobotArray, MarkingArray, MarkingSegment


"""
This module provides some methods needed for the ros environment,
e.g. methods to convert candidates to ROS messages or methods to modify the dynamic reconfigure objects.
"""

_cv_bridge = CvBridge()
_game_state = None

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

def build_bounding_box_2d(candidate):
    """
    Builds a BoundingBox2D message out of a vision Candidate

    :param candidate: A vision Candidate
    :return: BoundingBox2D message
    """
    return BoundingBox2D(
        size_x=float(candidate.get_width()),
        size_y=float(candidate.get_height()),
        center=Pose2D(
            x=float(candidate.get_center_x()),
            y=float(candidate.get_center_y()) 
        )
    )

def build_goal_post_array_msg(header, goal_post_msgs):
    """
    Builds a GoalpostArray message out of a list of Goalpost messages

    :param header: ros header of the new message. Mostly the header of the image
    :param goal_post_msgs: List of goal post messages
    :return: GoalpostArray message
    """
    # Create goalposts msg
    goal_posts_msg = GoalpostArray()
    # Add header
    goal_posts_msg.header.frame_id = header.frame_id
    goal_posts_msg.header.stamp = header.stamp
    # Add detected goal posts to the message
    goal_posts_msg.posts = goal_post_msgs
    return goal_posts_msg

def build_goal_post_msg(goalpost):
    """
    Builds a Goalpost message

    :param goalpost: goalpost candidate
    :return: Goalpost message
    """
    # Create a empty post message
    post_msg = Goalpost()
    post_msg.bb = build_bounding_box_2d(goalpost)
    if goalpost.get_rating() is not None:
        post_msg.confidence = float(goalpost.get_rating())
    return post_msg

def build_balls_msg(header, balls):
    """
    Builds a BallArray message out of a list of ball messages

    :param header: ros header of the new message. Mostly the header of the image
    :param balls: A list of Ball messages
    :return: BallArray msg
    """
    # create ball msg
    balls_msg = BallArray()
    # Set header
    balls_msg.header.frame_id = header.frame_id
    balls_msg.header.stamp = header.stamp
    # Add balls
    balls_msg.balls = balls
    return balls_msg

def build_ball_msg(ball_candidate):
    """
    Builds a Ball message

    :param ball_candidate: ball Candidate
    :return: Ball msg
    """
    # Create a empty ball message
    ball_msg = Ball()
    ball_msg.center.x = float(ball_candidate.get_center_x())
    ball_msg.center.y = float(ball_candidate.get_center_y())
    ball_msg.bb = build_bounding_box_2d(ball_candidate)
    if ball_candidate.get_rating() is not None:
            ball_msg.confidence = float(ball_candidate.get_rating())
    return ball_msg

def build_obstacle_array_msg(header, obstacles):
    """
    Builds a RobotArray message containing a list of Robot messages

    :param header: ros header of the new message. Mostly the header of the image
    :param obstacles: a list of Robot messages
    :return: RobotArray message
    """
    # Create obstacle msg
    obstacles_msg = RobotArray()
    # Add header
    obstacles_msg.header.frame_id = header.frame_id
    obstacles_msg.header.stamp = header.stamp
    # Add obstacles
    obstacles_msg.robots = obstacles
    return obstacles_msg

def build_obstacle_msg(obstacle, obstacle_color=None):
    """
    Builds a Robot msg of a detected obstacle of a certain color

    :param Candidate obstacle: Obstacle candidate
    :param GameState.team_color obstacle_color: Color of the obstacle, defaults to None
    :return: Robot msg
    """
    obstacle_msg = Robot()
    obstacle_msg.bb = build_bounding_box_2d(obstacle)
    obstacle_msg.player_number = Robot.TEAM_UNKNOWN 
    obstacle_msg.team = get_team_from_robot_color(obstacle_color)
    obstacle_msg.state = Robot.STATE_UNKNOWN
    obstacle_msg.facing = Robot.FACING_UNKNOWN
    if obstacle.get_rating() is not None:
        obstacle_msg.confidence = float(obstacle.get_rating())
    return obstacle_msg

def build_field_boundary_msg(header, field_boundary):
    """
    Builds a FieldBoundary message containing the field boundary.

    :param header: ros header of the new message. Mostly the header of the image
    :param field_boundary: List of tuples containing the field boundary points.
    :return: FieldBoundary message
    """
    # Create message
    field_boundary_msg = FieldBoundary()
    # Add header
    field_boundary_msg.header = header
    # Add field boundary points
    for point in field_boundary:
        p = Point2D()
        p.x = float(point[0])
        p.y = float(point[1])
        field_boundary_msg.points.append(p)
    return field_boundary_msg

def build_marking_array_msg(header, marking_segments):
    """
    Builds a MarkingArray message that consists of marking segments.

    :param header: ros header of the new message. Mostly the header of the image
    :param marking_segments: A list of MarkingSegment messages
    :return: Final MarkingArray message
    """
    # Create message
    marking_array_msg = MarkingArray()
    # Set header values
    marking_array_msg.header.frame_id = header.frame_id
    marking_array_msg.header.stamp = header.stamp
    # Set line segments
    marking_array_msg.segments = marking_segments
    return marking_array_msg

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

def convert_line_points_to_marking_segment_msgs(line_points):
    """
    Converts a list of linepoints in the form [(x,y), ...] into a list of MarkingSegmentInImage messages.

    :param line_points: A list of linepoints in the form [(x,y), ...]
    :return: A list of MarkingSegment messages
    """
    marking_segments = []
    for line_point in line_points:
        # Create MarkingSegment message
        marking_segment = MarkingSegment()
        marking_segment.start.x = float(line_point[0])
        marking_segment.start.y = float(line_point[1])
        marking_segment.end = marking_segment.start
        marking_segments.append(marking_segment)
    return marking_segments

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

def gamestate_callback(gamestate_msg):
    """
    This method is called by the GameState msg subscriber.
    """
    _game_state = gamestate_msg

def get_team_from_robot_color(color):
    """
    Maps the detected robot color to the current team.
    If the color is the same as the current team, returns own team, else returns opponent team.

    :param GameState.team_color color: Robot color
    :return Robot.team: Robot's team
    """
    if color not in [GameState.BLUE, GameState.RED]:  # If color is not known, we can just return unknown
        return Robot.TEAM_UNKNOWN

    # Color is known and we have to first figure out the own color
    # Get own team color
    own_color = None
    if _game_state is not None:
        own_color = _game_state.team_color

    if color == own_color:  # Robot is in own team, if same color
        return Robot.TEAM_OWN
    else:  # Robot is not same color, therefore it is from the opponent's team
        return Robot.TEAM_OPPONENT

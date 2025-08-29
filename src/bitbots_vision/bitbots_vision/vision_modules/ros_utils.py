import re
from typing import Optional, TypeAlias

from bitbots_utils.utils import get_parameters_from_other_node
from cv_bridge import CvBridge
from rclpy import logging
from rclpy.node import Node
from soccer_vision_2d_msgs.msg import (
    Ball,
    BallArray,
    Goalpost,
    GoalpostArray,
    MarkingArray,
    MarkingSegment,
    Robot,
    RobotArray,
)
from soccer_vision_attribute_msgs.msg import Robot as RobotAttributes
from vision_msgs.msg import BoundingBox2D, Pose2D

"""
This module provides some methods needed for the ros environment,
e.g. methods to convert candidates to ROS messages or methods to modify the dynamic reconfigure objects.
"""

_cv_bridge = CvBridge()

logger = logging.get_logger("bitbots_vision")

general_parameters = []

global own_team_color
own_team_color = None

T_RobotAttributes_Team: TypeAlias = int  # Type for RobotAttributes.team


def create_or_update_publisher(
    node, old_config, new_config, publisher_object, topic_key, data_class, qos_profile=1, callback_group=None
):
    """
    Creates or updates a publisher

    :param node: ROS node to which the publisher is bound
    :param old_config: Previous config dict
    :param new_config: Current config dict
    :param publisher_object: The python object, that represents the publisher
    :param topic_key: The name of the topic variable in the config dict
    :param data_class: Data type class for ROS messages of the topic we want to subscribe
    :param qos_profile: A QoSProfile or a history depth to apply to the publisher.
        In the case that a history depth is provided, the QoS history is set to
        KEEP_LAST, the QoS history depth is set to the value
        of the parameter, and all other QoS settings are set to their default values.
        Reference: https://github.com/ros2/rclpy/blob/6f7cfd0c73bda1afefba36b6785516f343d6b634/rclpy/rclpy/node.py#L1258
    :param callback_group: The callback group for the publisher's event handlers.
        If ``None``, then the node's default callback group is used.
        Reference: https://github.com/ros2/rclpy/blob/6f7cfd0c73bda1afefba36b6785516f343d6b634/rclpy/rclpy/node.py#L1262
    :return: adjusted publisher object
    """
    # Check if topic parameter has changed
    if config_param_change(old_config, new_config, topic_key):
        # Create the new publisher
        publisher_object = node.create_publisher(
            data_class, new_config[topic_key], qos_profile, callback_group=callback_group
        )
        logger.debug("Registered new publisher to " + str(new_config[topic_key]))
    return publisher_object


def create_or_update_subscriber(
    node, old_config, new_config, subscriber_object, topic_key, data_class, callback, qos_profile=1, callback_group=None
):
    """
    Creates or updates a subscriber

    :param node: ROS node to which the publisher is bound
    :param old_config: Previous config dict
    :param new_config: Current config dict
    :param subscriber_object: The python object, that represents the subscriber
    :param topic_key: The name of the topic variable in the config dict
    :param data_class: Data type class for ROS messages of the topic we want to subscribe
    :param callback: The subscriber callback function
    :param qos_profile: A QoSProfile or a history depth to apply to the subscription.
        In the case that a history depth is provided, the QoS history is set to
        KEEP_LAST, the QoS history depth is set to the value
        of the parameter, and all other QoS settings are set to their default values.
        Reference: https://github.com/ros2/rclpy/blob/6f7cfd0c73bda1afefba36b6785516f343d6b634/rclpy/rclpy/node.py#L1335
    :param callback_group: The callback group for the subscription. If ``None``, then the
        nodes default callback group is used.
        Reference: https://github.com/ros2/rclpy/blob/6f7cfd0c73bda1afefba36b6785516f343d6b634/rclpy/rclpy/node.py#L1339
    :return: adjusted subscriber object
    """
    # Check if topic parameter has changed
    if config_param_change(old_config, new_config, topic_key):
        # Create the new subscriber
        subscriber_object = node.create_subscription(
            data_class, new_config[topic_key], callback, qos_profile, callback_group=callback_group
        )
        logger.debug("Registered new subscriber at " + str(new_config[topic_key]))
    return subscriber_object


def build_bounding_box_2d(candidate):
    """
    Builds a BoundingBox2D message out of a vision Candidate

    :param candidate: A vision Candidate
    :return: BoundingBox2D message
    """
    center = Pose2D()
    center.position.x = float(candidate.get_center_x())
    center.position.y = float(candidate.get_center_y())

    bb_msg = BoundingBox2D()
    bb_msg.size_x = float(candidate.get_width())
    bb_msg.size_y = float(candidate.get_height())
    bb_msg.center = center
    return bb_msg


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
    goal_posts_msg.header = header
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
        post_msg.confidence.confidence = float(goalpost.get_rating())
    return post_msg


def build_ball_array_msg(header, balls):
    """
    Builds a BallArray message out of a list of ball messages

    :param header: ros header of the new message. Mostly the header of the image
    :param balls: A list of Ball messages
    :return: BallArray msg
    """
    # create ball msg
    balls_msg = BallArray()
    # Set header
    balls_msg.header = header
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
    ball_msg.bb = build_bounding_box_2d(ball_candidate)
    ball_msg.center.x = float(ball_candidate.get_center_x())
    ball_msg.center.y = float(ball_candidate.get_center_y())
    if ball_candidate.get_rating() is not None:
        ball_msg.confidence.confidence = float(ball_candidate.get_rating())
    return ball_msg


def build_robot_array_msg(header, robots):
    """
    Builds a RobotArray message containing a list of Robot messages

    :param header: ros header of the new message. Mostly the header of the image
    :param robots: a list of Robot messages
    :return: RobotArray message
    """
    # Create obstacle msg
    robots_msg = RobotArray()
    # Add header
    robots_msg.header = header
    # Add obstacles
    robots_msg.robots = robots
    return robots_msg


def build_robot_msg(obstacle, obstacle_color=None):
    """
    Builds a Robot msg of a detected obstacle of a certain color

    :param Candidate obstacle: Obstacle candidate
    :param GameState.team_color obstacle_color: Color of the obstacle, defaults to None
    :return: Robot msg
    """
    obstacle_msg = Robot()
    obstacle_msg.bb = build_bounding_box_2d(obstacle)
    obstacle_msg.attributes.team = get_team_from_robot_color(obstacle_color)
    if obstacle.get_rating() is not None:
        obstacle_msg.confidence.confidence = float(obstacle.get_rating())
    return obstacle_msg


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
    marking_array_msg.header = header
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


def set_general_parameters(params):
    """
    Sets params, that should trigger every `config_param_change` call.

    :params list of global params
    """
    general_parameters.extend(params)


def config_param_change(old_config, new_config, params_expressions, check_generals=True):
    """
    Checks whether some of the specified config params have changed.

    :param dict old_config: old config dict
    :param dict new_config: new config dict
    :param list of str or str params_expressions: regex describing parameter name or list of parameter names
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


def update_own_team_color(vision_node: Node):
    global own_team_color
    params = get_parameters_from_other_node(
        vision_node, "parameter_blackboard", ["team_color"], service_timeout_sec=2.0
    )
    own_team_color = params["team_color"]
    vision_node._logger.debug(f"Own team color is: {own_team_color}")


def get_team_from_robot_color(color: int) -> T_RobotAttributes_Team:
    """
    Maps the detected robot color to the current team.
    If the color is the same as the current team, returns own team, else returns opponent team.
    """
    global own_team_color
    if own_team_color is not None and 0 >= own_team_color <= 1:
        return RobotAttributes.TEAM_UNKNOWN

    if 0 >= color <= 1:  # If color is not known, we can just return unknown
        return Robot().attributes.TEAM_UNKNOWN

    if color == own_team_color:  # Robot is in own team, if same color
        return RobotAttributes.TEAM_OWN
    else:  # Robot is not same color, therefore it is from the opponent's team
        return RobotAttributes.TEAM_OPPONENT


def get_robot_color_for_team(team: T_RobotAttributes_Team) -> Optional[int]:
    """
    Maps team (own, opponent, unknown) to the current robot color.
    """
    global own_team_color
    if own_team_color is None:
        return None  # This gets handled later

    if team == RobotAttributes.TEAM_OWN:
        return own_team_color
    elif team == RobotAttributes.TEAM_OPPONENT:
        if own_team_color == 0:  # 0 is blue, 1 is red
            return 1
        else:
            return 0
    else:
        return None

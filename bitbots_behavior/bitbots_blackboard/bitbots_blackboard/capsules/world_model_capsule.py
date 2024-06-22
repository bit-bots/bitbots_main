import math
from typing import TYPE_CHECKING, Dict, Optional, Tuple

if TYPE_CHECKING:
    from bitbots_blackboard.blackboard import BodyBlackboard

import numpy as np
import tf2_ros as tf2
from bitbots_utils.utils import get_parameter_dict, get_parameters_from_other_node
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    TransformStamped,
    TwistStamped,
)
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.time import Time
from ros2_numpy import msgify, numpify
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_geometry_msgs import Point, PointStamped
from tf_transformations import euler_from_quaternion


class WorldModelCapsule:
    """Provides information about the world model."""

    def __init__(self, blackboard: "BodyBlackboard"):
        self._blackboard = blackboard

        # Init Parameters
        # Get global parameters
        parameters = get_parameters_from_other_node(
            self._blackboard.node,
            "/parameter_blackboard",
            [
                "field.goal.width",
                "field.markings.penalty_area.size.x",
                "field.size.x",
                "field.size.y",
            ],
        )
        self.goal_width: float = parameters["field.goal.width"]
        self.penalty_area_size_x: float = parameters["field.markings.penalty_area.size.x"]
        self.field_length: float = parameters["field.size.x"]
        self.field_width: float = parameters["field.size.y"]

        # Define the frames
        self.base_footprint_frame: str = self._blackboard.node.get_parameter("base_footprint_frame").value
        self.map_frame: str = self._blackboard.node.get_parameter("map_frame").value

        # Get body parameters
        self.ball_covariance_threshold = self._blackboard.node.get_parameter("ball_max_covariance").value
        self.ball_position_precision_threshold = get_parameter_dict(
            self._blackboard.node, "body.ball_position_precision_threshold"
        )
        self.ball_twist_lost_time = Duration(
            seconds=self._blackboard.node.get_parameter("body.ball_twist_lost_time").value
        )
        self.ball_twist_precision_threshold = get_parameter_dict(
            self._blackboard.node, "body.ball_twist_precision_threshold"
        )
        self.localization_precision_threshold: Dict[str, float] = get_parameter_dict(
            self._blackboard.node, "body.localization_precision_threshold"
        )
        self.map_margin: float = self._blackboard.node.get_parameter("body.map_margin").value

        # Placeholders
        self.ball_base_footprint: PoseWithCovarianceStamped | None = None  # The ball in the base footprint frame
        self.ball_map: PoseWithCovarianceStamped | None = (
            None  # The ball in the map frame (when localization is usable)
        )
        self.ball_twist_map: TwistStamped | None = None
        self.pose: PoseWithCovarianceStamped | None = None  # Own pose with covariance

        # Publisher for debug messages
        self.debug_publisher_ball = self._blackboard.node.create_publisher(
            PoseWithCovarianceStamped, "debug/viz_ball", 1
        )
        self.debug_publisher_ball_twist = self._blackboard.node.create_publisher(TwistStamped, "debug/ball_twist", 1)
        self.debug_publisher_used_ball = self._blackboard.node.create_publisher(PointStamped, "debug/used_ball", 1)
        self.debug_publisher_which_ball = self._blackboard.node.create_publisher(Header, "debug/which_ball_is_used", 1)

        # Services
        self.reset_ball_filter = self._blackboard.node.create_client(Trigger, "ball_filter_reset")

        # make subscription
        self._blackboard.node.create_subscription(
            PoseWithCovarianceStamped,
            "ball_position_relative_filtered",
            self.ball_filtered_callback,
            1,
        )

    ############
    ### Ball ###
    ############

    def ball_has_been_seen(self) -> bool:
        """Returns true if we or a teammate have seen the ball recently (less than ball_lost_time ago)"""
        return np.sum(self.ball_map.pose.covariance) < self.ball_covariance_threshold

    def get_ball_position_xy(self) -> Tuple[float, float]:
        """Return the ball saved in the map or odom frame"""
        ball = self.ball_map.pose
        return ball.point.x, ball.point.y

    def get_ball_position_uv(self) -> Tuple[float, float]:
        """
        Returns the ball position relative to the robot in the base_footprint frame
        """
        ball_bfp = self.ball_base_footprint.pose
        return ball_bfp.x, ball_bfp.y

    def get_ball_distance(self) -> float:
        ball_pos = self.get_ball_position_uv()
        if ball_pos is None:
            return np.inf  # worst case (very far away)
        else:
            u, v = ball_pos

        return math.hypot(u, v)

    def get_ball_angle(self) -> float:
        ball_pos = self.get_ball_position_uv()
        if ball_pos is None:
            return -math.pi  # worst case (behind robot)
        else:
            u, v = ball_pos
        return math.atan2(v, u)

    def ball_filtered_callback(self, msg: PoseWithCovarianceStamped):
        ball_buffer = PoseStamped(header=msg.header, pose=msg.pose.pose)
        try:
            ball_base_footprint = self._blackboard.tf_buffer.transform(
                ball_buffer, self.base_footprint_frame, timeout=Duration(seconds=1.0)
            )
            ball_map = self._blackboard.tf_buffer.transform(ball_buffer, self.map_frame, timeout=Duration(seconds=1.0))
            self.ball_base_footprint = PoseWithCovarianceStamped(
                header=ball_base_footprint.header,
                pose=PoseWithCovariance(pose=ball_base_footprint.pose, covariance=msg.pose.covariance),
            )
            self.ball_map = PoseWithCovarianceStamped(
                header=ball_map.header, pose=PoseWithCovariance(pose=ball_map.pose, covariance=msg.pose.covariance)
            )
            # Set timestamps to zero to get the newest transform when this is transformed later
            self.ball_base_footprint.header.stamp = Time(clock_type=ClockType.ROS_TIME).to_msg()
            self.ball_map.header.stamp = Time(clock_type=ClockType.ROS_TIME).to_msg()

            self.debug_publisher_ball.publish(self.ball_base_footprint)

        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self._blackboard.node.get_logger().warn(str(e))

    def forget_ball(self, own: bool = True, team: bool = True, reset_ball_filter: bool = True) -> None:
        """
        Forget that we and the best teammate saw a ball, optionally reset the ball filter
        :param own: Forget the ball recognized by the own robot, defaults to True
        :type own: bool, optional
        :param team: Forget the ball received from the team, defaults to True
        :type team: bool, optional
        :param reset_ball_filter: Reset the ball filter, defaults to True
        :type reset_ball_filter: bool, optional
        """
        if own:  # Forget own ball
            self._ball_seen_time = Time(clock_type=ClockType.ROS_TIME)
            self.ball_base_footprint = PoseWithCovarianceStamped()
            self.ball_map = PoseWithCovarianceStamped()

        if reset_ball_filter:  # Reset the ball filter
            result: Trigger.Response = self.reset_ball_filter.call(Trigger.Request())
            if result.success:
                self._blackboard.node.get_logger().debug(f"Received message from ball filter: '{result.message}'")
            else:
                self._blackboard.node.get_logger().warn(f"Ball filter reset failed with: '{result.message}'")

    ########
    # Goal #
    ########

    def get_map_based_opp_goal_center_uv(self):
        x, y = self.get_map_based_opp_goal_center_xy()
        return self.get_uv_from_xy(x, y)

    def get_map_based_opp_goal_center_xy(self):
        return self.field_length / 2, 0.0

    def get_map_based_own_goal_center_uv(self):
        x, y = self.get_map_based_own_goal_center_xy()
        return self.get_uv_from_xy(x, y)

    def get_map_based_own_goal_center_xy(self):
        return -self.field_length / 2, 0.0

    def get_map_based_opp_goal_angle_from_ball(self):
        ball_x, ball_y = self.get_ball_position_xy()
        goal_x, goal_y = self.get_map_based_opp_goal_center_xy()
        return math.atan2(goal_y - ball_y, goal_x - ball_x)

    def get_map_based_opp_goal_distance(self):
        x, y = self.get_map_based_opp_goal_center_xy()
        return self.get_distance_to_xy(x, y)

    def get_map_based_opp_goal_angle(self):
        x, y = self.get_map_based_opp_goal_center_uv()
        return math.atan2(y, x)

    def get_map_based_opp_goal_left_post_uv(self):
        x, y = self.get_map_based_opp_goal_center_xy()
        return self.get_uv_from_xy(x, y - self.goal_width / 2)

    def get_map_based_opp_goal_right_post_uv(self):
        x, y = self.get_map_based_opp_goal_center_xy()
        return self.get_uv_from_xy(x, y + self.goal_width / 2)

    ########
    # Pose #
    ########

    def pose_callback(self, pos: PoseWithCovarianceStamped):
        self.pose = pos

    def get_current_position(self, frame_id: Optional[str] = None) -> Optional[Tuple[float, float, float]]:
        """
        Returns the current position as determined by the localization or odometry
        depending on the given frame_id
        :param frame_id: The frame_id to use for the position (e.g. 'map' or 'odom'), will default to map_frame
        :returns x,y,theta:
        """
        transform = self.get_current_position_transform(frame_id or self.map_frame)
        if transform is None:
            return None
        theta = euler_from_quaternion(numpify(transform.transform.rotation))[2]
        return transform.transform.translation.x, transform.transform.translation.y, theta

    def get_current_position_pose_stamped(self, frame_id: Optional[str] = None) -> Optional[PoseStamped]:
        """
        Returns the current position as determined by the localization or odometry as a PoseStamped
        depending on the given frame_id
        :param frame_id: The frame_id to use for the position (e.g. 'map' or 'odom'), will default to map_frame
        """
        transform = self.get_current_position_transform(frame_id or self.map_frame)
        if transform is None:
            return None
        ps = PoseStamped()
        ps.header = transform.header
        ps.pose.position = msgify(Point, numpify(transform.transform.translation))
        ps.pose.orientation.x = transform.transform.rotation.x
        ps.pose.orientation.y = transform.transform.rotation.y
        ps.pose.orientation.z = transform.transform.rotation.z
        ps.pose.orientation.w = transform.transform.rotation.w
        return ps

    def get_current_position_transform(self, frame_id: str) -> TransformStamped:
        """
        Returns the current position as determined by the localization or odometry as a TransformStamped
        depending on the given frame_id
        :param frame_id: The frame_id to use for the position (e.g. 'map' or 'odom'), will default to map_frame
        """
        try:
            return self._blackboard.tf_buffer.lookup_transform(
                frame_id, self.base_footprint_frame, Time(clock_type=ClockType.ROS_TIME)
            )
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            self._blackboard.node.get_logger().warn(str(e))
            return None

    def get_localization_precision(self) -> Tuple[float, float, float]:
        """
        Returns the current localization precision based on the covariance matrix.
        """
        x_sdev = self.pose.pose.covariance[0]  # position 0,0 in a 6x6-matrix
        y_sdev = self.pose.pose.covariance[7]  # position 1,1 in a 6x6-matrix
        theta_sdev = self.pose.pose.covariance[35]  # position 5,5 in a 6x6-matrix
        return (x_sdev, y_sdev, theta_sdev)

    def localization_precision_in_threshold(self) -> bool:
        """
        Returns whether the last localization precision values were in the threshold defined in the settings.
        """
        # Check whether we can transform into and from the map frame seconds.
        if not self.localization_pose_current():
            return False
        # get the standard deviation values of the covariance matrix
        precision = self.get_localization_precision()
        # return whether those values are in the threshold
        return (
            precision[0] < self.localization_precision_threshold["x_sdev"]
            and precision[1] < self.localization_precision_threshold["y_sdev"]
            and precision[2] < self.localization_precision_threshold["theta_sdev"]
        )

    def localization_pose_current(self) -> bool:
        """
        Returns whether we can transform into and from the map frame.
        """
        # if we can do this, we should be able to transform the ball
        # (unless the localization died during the last 1.0 seconds)
        try:
            t = self._blackboard.node.get_clock().now() - Duration(seconds=1.0)
        except ValueError as e:
            self._blackboard.node.get_logger().error(str(e))
            t = Time(clock_type=ClockType.ROS_TIME)
        return self._blackboard.tf_buffer.can_transform(self.base_footprint_frame, self.map_frame, t)

    ##########
    # Common #
    ##########

    def get_uv_from_xy(self, x, y) -> Tuple[float, float]:
        """Returns the relativ positions of the robot to this absolute position"""
        current_position = self.get_current_position()
        x2 = x - current_position[0]
        y2 = y - current_position[1]
        theta = -1 * current_position[2]
        u = math.cos(theta) * x2 + math.sin(theta) * y2
        v = math.cos(theta) * y2 - math.sin(theta) * x2
        return u, v

    def get_xy_from_uv(self, u, v):
        """Returns the absolute position from the given relative position to the robot"""
        pos_x, pos_y, theta = self.get_current_position()
        angle = math.atan2(v, u) + theta
        hypotenuse = math.hypot(u, v)
        return pos_x + math.sin(angle) * hypotenuse, pos_y + math.cos(angle) * hypotenuse

    def get_distance_to_xy(self, x, y):
        """Returns distance from robot to given position"""
        u, v = self.get_uv_from_xy(x, y)
        dist = math.hypot(u, v)
        return dist

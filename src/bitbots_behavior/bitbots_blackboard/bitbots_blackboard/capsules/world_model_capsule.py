import math
from typing import Optional

import numpy as np
import tf2_ros as tf2
from bitbots_utils.utils import get_parameters_from_other_node
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from rclpy.clock import ClockType
from rclpy.time import Time
from ros2_numpy import msgify, numpify
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_geometry_msgs import Point, PointStamped
from tf_transformations import euler_from_quaternion

from bitbots_blackboard.capsules import AbstractBlackboardCapsule, cached_capsule_function


class WorldModelTFError(Exception): ...


class WorldModelPositionTFError(WorldModelTFError): ...


class WorldModelCapsule(AbstractBlackboardCapsule):
    """Provides information about the world model."""

    def __init__(self, node, blackboard):
        super().__init__(node, blackboard)

        # Init Parameters
        # Get global parameters
        parameters = get_parameters_from_other_node(
            self._node,
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
        self.base_footprint_frame: str = self._node.get_parameter("base_footprint_frame").value
        self.map_frame: str = self._node.get_parameter("map_frame").value

        # Get body parameters
        self.ball_max_covariance = self._node.get_parameter("ball_max_covariance").value
        self.map_margin: float = self._node.get_parameter("map_margin").value

        # Ball state
        # The ball this robot observed itself and the ball that is fused from the observations of
        # the whole team. Both are stored in the map frame and default to the center of the field
        # with an unknown covariance, as we did not see any ball yet.
        self._ball: PointStamped = self._unknown_ball()
        self._ball_covariance: np.ndarray = self._unknown_ball_covariance()
        self._team_ball: PointStamped = self._unknown_ball()
        self._team_ball_covariance: np.ndarray = self._unknown_ball_covariance()

        # Publisher for visualization in RViZ
        self.debug_publisher_own_ball = self._node.create_publisher(PointStamped, "debug/behavior/own_ball", 1)
        self.debug_publisher_team_ball = self._node.create_publisher(PointStamped, "debug/behavior/team_ball", 1)

        # Services
        self.reset_ball_filter = self._node.create_client(Trigger, "ball_filter_reset")

    ############
    ### Ball ###
    ############

    # The behavior distinguishes two balls. The ball this robot observed itself is precise enough
    # to play it, but it is only available while we actually look at the ball. The team ball is
    # fused from the observations of all robots of our team, so it is available far more often,
    # but it may be based on the observation of a robot that stands somewhere else on the field.
    # Therefore the team ball is used to decide who plays the ball, where the other robots position
    # themselves and whether the team needs to search for the ball, while everything that moves the
    # robot relative to the ball uses the ball this robot observed itself.

    def _unknown_ball(self) -> PointStamped:
        """Returns a ball in the center of the field, used as long as we do not know better"""
        return PointStamped(header=Header(stamp=Time(clock_type=ClockType.ROS_TIME).to_msg(), frame_id=self.map_frame))

    def _unknown_ball_covariance(self) -> np.ndarray:
        """Returns the covariance of a ball we know nothing about"""
        return np.eye(2) * self.ball_max_covariance * 1000

    @cached_capsule_function
    def ball_seen(self) -> bool:
        """Returns true if this robot is reasonably sure that it observed the ball itself"""
        return bool(np.all(np.diag(self._ball_covariance) < self.ball_max_covariance))

    @cached_capsule_function
    def team_ball_seen(self) -> bool:
        """Returns true if anybody in our team is reasonably sure where the ball is"""
        return bool(np.all(np.diag(self._team_ball_covariance) < self.ball_max_covariance))

    def get_ball_position_xy(self) -> tuple[float, float]:
        """Returns the absolute position of the ball this robot observed itself on the field"""
        return self._ball.point.x, self._ball.point.y

    def get_team_ball_position_xy(self) -> tuple[float, float]:
        """Returns the absolute position of the team ball on the field"""
        return self._team_ball.point.x, self._team_ball.point.y

    @cached_capsule_function
    def get_ball_position_uv(self) -> tuple[float, float]:
        """
        Returns the position of the ball this robot observed itself relative to the robot
        in the base_footprint frame.
        U and V are returned, where positive U is forward, positive V is to the left.
        """
        return self._get_position_uv(self._ball)

    @cached_capsule_function
    def get_team_ball_position_uv(self) -> tuple[float, float]:
        """
        Returns the position of the team ball relative to the robot in the base_footprint frame.
        U and V are returned, where positive U is forward, positive V is to the left.
        """
        return self._get_position_uv(self._team_ball)

    def _get_position_uv(self, ball: PointStamped) -> tuple[float, float]:
        """Transforms a ball in the map frame into a position relative to the robot"""
        assert ball.header.frame_id == self.map_frame, "Ball needs to be in the map frame"
        our_pose = self.get_current_position_transform()
        assert our_pose.header.frame_id == self.map_frame, "Our pose needs to be in the map frame"
        # Construct the homogeneous transformation matrix for the ball position
        ball_transform = np.eye(4)
        ball_transform[0, 3] = ball.point.x
        ball_transform[1, 3] = ball.point.y
        # Get the homogeneous transformation matrix for the robot's current position
        our_pose_transform = numpify(our_pose.transform)
        # Calculate the relative position of the ball to the robot
        relative_transform = np.linalg.inv(our_pose_transform) @ ball_transform
        return relative_transform[0, 3], relative_transform[1, 3]

    def get_ball_distance(self) -> float:
        """
        Returns the distance to the ball this robot observed itself in meters.
        """
        u, v = self.get_ball_position_uv()
        return math.hypot(u, v)

    def get_team_ball_distance(self) -> float:
        """
        Returns the distance to the team ball in meters.
        """
        u, v = self.get_team_ball_position_uv()
        return math.hypot(u, v)

    def get_ball_angle(self) -> float:
        """
        Returns the angle to the ball this robot observed itself in radians.
        0 is straight ahead, positive is to the left, negative is to the right.
        """
        u, v = self.get_ball_position_uv()
        return math.atan2(v, u)

    def get_team_ball_angle(self) -> float:
        """
        Returns the angle to the team ball in radians.
        0 is straight ahead, positive is to the left, negative is to the right.
        """
        u, v = self.get_team_ball_position_uv()
        return math.atan2(v, u)

    def ball_filtered_callback(self, msg: PoseWithCovarianceStamped):
        """
        Handles incoming messages about the ball this robot observed itself
        """
        ball = self._ball_to_map_frame(msg)
        if ball is None:
            return
        self._ball = ball
        # Save covariance (only x and y parts)
        self._ball_covariance = msg.pose.covariance.reshape((6, 6))[:2, :2]
        self.debug_publisher_own_ball.publish(ball)

    def team_ball_filtered_callback(self, msg: PoseWithCovarianceStamped):
        """
        Handles incoming messages about the ball that is fused from all observations of our team
        """
        ball = self._ball_to_map_frame(msg)
        if ball is None:
            return
        self._team_ball = ball
        # Save covariance (only x and y parts)
        self._team_ball_covariance = msg.pose.covariance.reshape((6, 6))[:2, :2]
        self.debug_publisher_team_ball.publish(ball)

    def _ball_to_map_frame(self, msg: PoseWithCovarianceStamped) -> Optional[PointStamped]:
        """
        Converts an incoming ball message into a point in the map frame
        """
        ball = PointStamped(
            header=Header(
                # Set timestamps to zero to get the newest transform when this is transformed later
                stamp=Time(clock_type=ClockType.ROS_TIME).to_msg(),
                frame_id=msg.header.frame_id,
            ),
            point=msg.pose.pose.position,
        )

        # transform ball to map frame if it is not already in the map frame
        try:
            return self._blackboard.tf_buffer.transform(ball, self.map_frame)
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self._node.get_logger().warn(str(e))
            return None

    def forget_ball(self) -> None:
        """
        Forget that we saw a ball, both our own and the team ball
        """
        self.reset_ball_filter.call_async(Trigger.Request())

    ########
    # Goal #
    ########

    def get_map_based_opp_goal_center_uv(self) -> tuple[float, float]:
        x, y = self.get_map_based_opp_goal_center_xy()
        return self.get_uv_from_xy(x, y)

    def get_map_based_opp_goal_center_xy(self) -> tuple[float, float]:
        return self.field_length / 2, 0.0

    def get_map_based_own_goal_center_uv(self) -> tuple[float, float]:
        x, y = self.get_map_based_own_goal_center_xy()
        return self.get_uv_from_xy(x, y)

    def get_map_based_own_goal_center_xy(self) -> tuple[float, float]:
        return -self.field_length / 2, 0.0

    def get_map_based_opp_goal_distance(self) -> float:
        x, y = self.get_map_based_opp_goal_center_xy()
        return self.get_distance_to_xy(x, y)

    def get_map_based_opp_goal_angle(self) -> float:
        x, y = self.get_map_based_opp_goal_center_uv()
        return math.atan2(y, x)

    def get_map_based_opp_goal_left_post_uv(self) -> tuple[float, float]:
        x, y = self.get_map_based_opp_goal_center_xy()
        return self.get_uv_from_xy(x, y - self.goal_width / 2)

    def get_map_based_opp_goal_right_post_uv(self) -> tuple[float, float]:
        x, y = self.get_map_based_opp_goal_center_xy()
        return self.get_uv_from_xy(x, y + self.goal_width / 2)

    ########
    # Pose #
    ########

    @cached_capsule_function
    def get_current_position(self) -> tuple[float, float, float]:
        """
        Returns the current position on the field as determined by the localization.
        0,0,0 is the center of the field looking in the direction of the opponent goal.
        :returns x,y,theta:
        """
        transform = self.get_current_position_transform()
        theta = euler_from_quaternion(numpify(transform.transform.rotation))[2]
        return transform.transform.translation.x, transform.transform.translation.y, theta

    @cached_capsule_function
    def get_current_position_pose_stamped(self) -> PoseStamped:
        """
        Returns the current position as determined by the localization as a PoseStamped
        """
        transform = self.get_current_position_transform()
        return PoseStamped(
            header=transform.header,
            pose=Pose(
                position=msgify(Point, numpify(transform.transform.translation)),
                orientation=transform.transform.rotation,
            ),
        )

    @cached_capsule_function
    def get_current_position_transform(self) -> TransformStamped:
        """
        Returns the current position as determined by the localization as a TransformStamped
        """
        try:
            return self._blackboard.tf_buffer.lookup_transform(
                self.map_frame, self.base_footprint_frame, Time(clock_type=ClockType.ROS_TIME)
            )
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            self._node.get_logger().warn(str(e))
            raise WorldModelPositionTFError("Could not get current position transform") from e

    ##########
    # Common #
    ##########

    def get_uv_from_xy(self, x: float, y: float) -> tuple[float, float]:
        """Transforms a map position in the map frame into a relative position to the robot in the base_footprint frame"""
        current_position = self.get_current_position()
        x2 = x - current_position[0]
        y2 = y - current_position[1]
        theta = current_position[2]
        # Rotate the difference vector from the map frame into the robot frame
        u = math.cos(theta) * x2 + math.sin(theta) * y2
        v = -math.sin(theta) * x2 + math.cos(theta) * y2
        return u, v

    def get_xy_from_uv(self, u: float, v: float) -> tuple[float, float]:
        """Transforms a relative position to the robot in the base_footprint frame into an absolute position in the map frame"""
        pos_x, pos_y, theta = self.get_current_position()
        angle = math.atan2(v, u) + theta
        hypotenuse = math.hypot(u, v)
        return pos_x + math.cos(angle) * hypotenuse, pos_y + math.sin(angle) * hypotenuse

    def get_distance_to_xy(self, x: float, y: float) -> float:
        """Returns distance from robot to given position"""
        u, v = self.get_uv_from_xy(x, y)
        dist = math.hypot(u, v)
        return dist

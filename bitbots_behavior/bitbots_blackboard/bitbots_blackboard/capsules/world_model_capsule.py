import math

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
from rclpy.duration import Duration
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg
from ros2_numpy import msgify, numpify
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_geometry_msgs import Point, PointStamped
from tf_transformations import euler_from_quaternion

from bitbots_blackboard.capsules import AbstractBlackboardCapsule


class WorldModelTFError(Exception):
    ...


class WorldModelPositionTFError(WorldModelTFError):
    ...


class WorldModelBallTFError(WorldModelTFError):
    ...


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
        self._ball: PointStamped = PointStamped(
            header=Header(stamp=TimeMsg(), frame_id=self.map_frame)
        )  # The ball in the map frame (default to the center of the field if ball is not seen yet)
        self._ball_covariance: np.ndarray = np.zeros((2, 2))  # Covariance of the ball

        # Publisher for visualization in RViZ
        self.debug_publisher_used_ball = self._node.create_publisher(PointStamped, "debug/behavior/used_ball", 1)
        self.debug_publisher_which_ball = self._node.create_publisher(Header, "debug/behavior/which_ball_is_used", 1)

        # Services
        self.reset_ball_filter = self._node.create_client(Trigger, "ball_filter_reset")

    ############
    ### Ball ###
    ############

    def ball_seen_self(self) -> bool:
        """Returns true we are reasonably sure that we have seen the ball"""
        return all(np.diag(self._ball_covariance) < self.ball_max_covariance)

    def ball_has_been_seen(self) -> bool:
        """Returns true if we or a teammate are reasonably sure that we have seen the ball"""
        return self.ball_seen_self() or self._blackboard.team_data.teammate_ball_is_valid()

    def get_ball_position_xy(self) -> tuple[float, float]:
        """Return the ball saved in the map frame, meaning the absolute position of the ball on the field"""
        ball = self.get_best_ball_point_stamped()
        return ball.point.x, ball.point.y

    def get_best_ball_point_stamped(self) -> PointStamped:
        """
        Returns the best ball, either its own ball has been in the ball_lost_lost time
        or from teammate if the robot itself has lost it and teamcomm is available
        """
        # Get balls
        own_ball = self._ball
        teammate_ball = self._blackboard.team_data.get_teammate_ball()

        # If the robot has lost the ball and the teammate has seen it, use the teammate's ball
        if not self.ball_seen_self() and teammate_ball is not None and teammate_ball.header.frame_id == self.map_frame:
            self.debug_publisher_used_ball.publish(teammate_ball)
            self.debug_publisher_which_ball.publish(Header(stamp=teammate_ball.header.stamp, frame_id="teammate_ball"))
            return teammate_ball

        # Otherwise, use the own ball even if it is bad
        if not self.ball_seen_self():
            self._node.get_logger().warn("Using own ball even though it is bad, as no teammate ball is available")
        self.debug_publisher_used_ball.publish(own_ball)
        self.debug_publisher_which_ball.publish(Header(stamp=own_ball.header.stamp, frame_id="own_ball_map"))
        return own_ball

    def get_ball_position_uv(self) -> tuple[float, float]:
        """
        Returns the ball position relative to the robot in the base_footprint frame.
        U and V are returned, where positive U is forward, positive V is to the left.
        """
        ball = self.get_best_ball_point_stamped()
        try:
            ball_bfp = self._blackboard.tf_buffer.transform(
                ball, self.base_footprint_frame, timeout=Duration(seconds=0.2)
            ).point
        except tf2.ExtrapolationException as e:
            self._node.get_logger().warn(str(e))
            self._node.get_logger().error("Severe transformation problem concerning the ball!")
            raise WorldModelBallTFError("Could not transform ball to base_footprint frame") from e
        return ball_bfp.x, ball_bfp.y

    def get_ball_distance(self) -> float:
        """
        Returns the distance to the ball in meters.
        """
        if not (ball_pos := self.get_ball_position_uv()):
            return np.inf  # worst case (very far away)
        u, v = ball_pos
        return math.hypot(u, v)

    def get_ball_angle(self) -> float:
        """
        Returns the angle to the ball in radians.
        0 is straight ahead, positive is to the left, negative is to the right.
        """
        if not (ball_pos := self.get_ball_position_uv()):
            return -math.pi  # worst case (behind robot)
        u, v = ball_pos
        return math.atan2(v, u)

    def ball_filtered_callback(self, msg: PoseWithCovarianceStamped):
        """
        Handles incoming ball messages
        """
        assert msg.header.frame_id == self.map_frame, "Ball needs to be in the map frame"

        # Save ball
        self._ball = PointStamped(
            header=Header(
                # Set timestamps to zero to get the newest transform when this is transformed later
                stamp=Time(clock_type=ClockType.ROS_TIME).to_msg(),
                frame_id=self.map_frame,
            ),
            point=msg.pose.pose.position,
        )

        # Save covariance (only x and y parts)
        self._ball_covariance = msg.pose.covariance.reshape((6, 6))[:2, :2]

    def forget_ball(self) -> None:
        """
        Forget that we saw a ball
        """
        result: Trigger.Response = self.reset_ball_filter.call(Trigger.Request())
        if not result.success:
            self._node.get_logger().error(f"Ball filter reset failed with: '{result.message}'")

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

    def get_map_based_opp_goal_angle_from_ball(self) -> float:
        ball_x, ball_y = self.get_ball_position_xy()
        goal_x, goal_y = self.get_map_based_opp_goal_center_xy()
        return math.atan2(goal_y - ball_y, goal_x - ball_x)

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

    def get_current_position(self) -> tuple[float, float, float]:
        """
        Returns the current position on the field as determined by the localization.
        0,0,0 is the center of the field looking in the direction of the opponent goal.
        :returns x,y,theta:
        """
        transform = self.get_current_position_transform()
        theta = euler_from_quaternion(numpify(transform.transform.rotation))[2]
        return transform.transform.translation.x, transform.transform.translation.y, theta

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
        """Returns the relativ positions of the robot to this absolute position"""
        current_position = self.get_current_position()
        x2 = x - current_position[0]
        y2 = y - current_position[1]
        theta = -1 * current_position[2]
        u = math.cos(theta) * x2 + math.sin(theta) * y2
        v = math.cos(theta) * y2 - math.sin(theta) * x2
        return u, v

    def get_xy_from_uv(self, u: float, v: float) -> tuple[float, float]:
        """Returns the absolute position from the given relative position to the robot"""
        pos_x, pos_y, theta = self.get_current_position()
        angle = math.atan2(v, u) + theta
        hypotenuse = math.hypot(u, v)
        return pos_x + math.sin(angle) * hypotenuse, pos_y + math.cos(angle) * hypotenuse

    def get_distance_to_xy(self, x: float, y: float) -> float:
        """Returns distance from robot to given position"""
        u, v = self.get_uv_from_xy(x, y)
        dist = math.hypot(u, v)
        return dist

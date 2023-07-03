"""
WorldModelCapsule
^^^^^^^^^^^^^^^^^^

Provides information about the world model.
"""
import math
from typing import TYPE_CHECKING, Dict, Optional, Tuple

if TYPE_CHECKING:
    from bitbots_blackboard.blackboard import BodyBlackboard

import numpy as np
import tf2_ros as tf2
from bitbots_utils.utils import (get_parameter_dict,
                                 get_parameters_from_other_node)
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                               TransformStamped, TwistStamped,
                               TwistWithCovarianceStamped)
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_geometry_msgs import PointStamped
from tf_transformations import euler_from_quaternion


class WorldModelCapsule:
    def __init__(self, blackboard: "BodyBlackboard"):
        self._blackboard = blackboard
        self.body_config = get_parameter_dict(self._blackboard.node, "body")
        # This pose is not supposed to be used as robot pose. Just as precision measurement for the TF position.
        self.pose = PoseWithCovarianceStamped()

        self.odom_frame: str = self._blackboard.node.get_parameter('odom_frame').value
        self.map_frame: str = self._blackboard.node.get_parameter('map_frame').value
        self.ball_frame: str = self._blackboard.node.get_parameter('ball_frame').value
        self.base_footprint_frame: str = self._blackboard.node.get_parameter('base_footprint_frame').value

        self.ball = PointStamped()  # The ball in the base footprint frame
        self.ball_odom = PointStamped()  # The ball in the odom frame (when localization is not usable)
        self.ball_odom.header.stamp = Time(seconds=0, nanoseconds=0, clock_type=ClockType.ROS_TIME).to_msg()
        self.ball_odom.header.frame_id = self.odom_frame
        self.ball_map = PointStamped()  # The ball in the map frame (when localization is usable)
        self.ball_map.header.stamp = Time(seconds=0, nanoseconds=0, clock_type=ClockType.ROS_TIME).to_msg()
        self.ball_map.header.frame_id = self.map_frame
        self.ball_teammate = PointStamped()
        self.ball_teammate.header.stamp = Time(seconds=0, nanoseconds=0, clock_type=ClockType.ROS_TIME).to_msg()
        self.ball_teammate.header.frame_id = self.map_frame
        self.ball_lost_time = Duration(seconds=self._blackboard.node.get_parameter('body.ball_lost_time').value)
        self.ball_twist_map: Optional[TwistStamped] = None
        self.ball_filtered: Optional[PoseWithCovarianceStamped] = None
        self.ball_twist_lost_time = Duration(seconds=self._blackboard.node.get_parameter('body.ball_twist_lost_time').value)
        self.ball_twist_precision_threshold = get_parameter_dict(self._blackboard.node, 'body.ball_twist_precision_threshold')
        self.reset_ball_filter = self._blackboard.node.create_client(Trigger, 'ball_filter_reset')

        self.counter: int = 0
        self.ball_seen_time = Time(seconds=0, nanoseconds=0, clock_type=ClockType.ROS_TIME)
        self.ball_seen_time_teammate = Time(seconds=0, nanoseconds=0, clock_type=ClockType.ROS_TIME)
        self.ball_seen: bool = False
        self.ball_seen_teammate: bool = False
        parameters = get_parameters_from_other_node(
            self._blackboard.node,
            "/parameter_blackboard",
            ["field_length", "field_width", "goal_width"])
        self.field_length: float = parameters["field_length"]
        self.field_width: float = parameters["field_width"]
        self.goal_width: float = parameters["goal_width"]
        self.map_margin: float = self._blackboard.node.get_parameter('body.map_margin').value
        self.obstacle_costmap_smoothing_sigma: float = self._blackboard.node.get_parameter(
            'body.obstacle_costmap_smoothing_sigma').value
        self.obstacle_cost: float = self._blackboard.node.get_parameter('body.obstacle_cost').value

        self.localization_precision_threshold: Dict[str, float] = get_parameter_dict(
            self._blackboard.node, 'body.localization_precision_threshold')

        # Publisher for visualization in RViZ
        self.ball_publisher = self._blackboard.node.create_publisher(PointStamped, 'debug/viz_ball', 1)
        self.ball_twist_publisher = self._blackboard.node.create_publisher(TwistStamped, 'debug/ball_twist', 1)
        self.used_ball_pub = self._blackboard.node.create_publisher(PointStamped, 'debug/used_ball', 1)
        self.which_ball_pub = self._blackboard.node.create_publisher(Header, 'debug/which_ball_is_used', 1)


    ############
    ### Ball ###
    ############

    def ball_seen_self(self) -> bool:
        """Returns true if we have seen the ball recently (less than ball_lost_time ago)"""
        return self._blackboard.node.get_clock().now() - self.ball_seen_time < self.ball_lost_time

    def ball_last_seen(self) -> Time:
        """
        Returns the time at which the ball was last seen if it is in the threshold or
        the more recent ball from either the teammate or itself if teamcom is available
        """
        if not self.ball_seen_self() and \
                (hasattr(self._blackboard, "team_data") and self.localization_precision_in_threshold()):
            return max(self.ball_seen_time, self._blackboard.team_data.get_teammate_ball_seen_time())
        else:
            return self.ball_seen_time

    def ball_has_been_seen(self) -> bool:
        """Returns true if we or a teammate have seen the ball recently (less than ball_lost_time ago)"""
        return self._blackboard.node.get_clock().now() - self.ball_last_seen() < self.ball_lost_time

    def get_ball_position_xy(self) -> Tuple[float, float]:
        """Return the ball saved in the map or odom frame"""
        ball = self.get_best_ball_point_stamped()
        return ball.point.x, ball.point.y

    def get_ball_stamped_relative(self) -> PointStamped:
        """ Returns the ball in the base_footprint frame i.e. relative to the robot projected on the ground"""
        return self.ball

    def get_best_ball_point_stamped(self) -> PointStamped:
        """
        Returns the best ball, either its own ball has been in the ball_lost_lost time
        or from teammate if the robot itself has lost it and teamcom is available
        """
        if self.localization_precision_in_threshold():
            if self.ball_seen_self() or not hasattr(self._blackboard, "team_data"):
                self.used_ball_pub.publish(self.ball_map)
                h = Header()
                h.stamp = self.ball_map.header.stamp
                h.frame_id = "own_ball_map"
                self.which_ball_pub.publish(h)
                return self.ball_map
            else:
                teammate_ball = self._blackboard.team_data.get_teammate_ball()
                if teammate_ball is not None and self._blackboard.tf_buffer.can_transform(self.base_footprint_frame,
                                                                              teammate_ball.header.frame_id,
                                                                              teammate_ball.header.stamp,
                                                                              timeout=Duration(seconds=0.2)):
                    self.used_ball_pub.publish(teammate_ball)
                    h = Header()
                    h.stamp = teammate_ball.header.stamp
                    h.frame_id = "teammate_ball"
                    self.which_ball_pub.publish(h)
                    return teammate_ball
                else:
                    self._blackboard.node.get_logger().warning(
                        "our ball is bad but the teammates ball is worse or cant be transformed")
                    h = Header()
                    h.stamp = self.ball_map.header.stamp
                    h.frame_id = "own_ball_map"
                    self.which_ball_pub.publish(h)
                    self.used_ball_pub.publish(self.ball_map)
                    return self.ball_map
        else:
            h = Header()
            h.stamp = self.ball_odom.header.stamp
            h.frame_id = "own_ball_odom"
            self.which_ball_pub.publish(h)
            self.used_ball_pub.publish(self.ball_odom)
            return self.ball_odom

    def get_ball_position_uv(self) -> Tuple[float, float]:
        ball = self.get_best_ball_point_stamped()
        try:
            ball_bfp = self._blackboard.tf_buffer.transform(ball, self.base_footprint_frame, timeout=Duration(seconds=0.2)).point
        except (tf2.ExtrapolationException) as e:
            self._blackboard.node.get_logger().warn(e)
            self._blackboard.node.get_logger().error('Severe transformation problem concerning the ball!')
            return None
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

    def get_ball_speed(self) -> TwistStamped:
        raise NotImplementedError

    def ball_filtered_callback(self, msg: PoseWithCovarianceStamped):
        self.ball_filtered = msg

        # When the precision is not sufficient, the ball ages.
        x_sdev = msg.pose.covariance[0]  # position 0,0 in a 6x6-matrix
        y_sdev = msg.pose.covariance[7]  # position 1,1 in a 6x6-matrix
        if x_sdev > self.body_config['ball_position_precision_threshold']['x_sdev'] or \
                y_sdev > self.body_config['ball_position_precision_threshold']['y_sdev']:
            self.forget_ball(own=True, team=False, reset_ball_filter=False)
            return

        ball_buffer = PointStamped(header=msg.header, point=msg.pose.pose.position)
        try:
            self.ball = self._blackboard.tf_buffer.transform(ball_buffer, self.base_footprint_frame, timeout=Duration(seconds=1.0))
            self.ball_odom = self._blackboard.tf_buffer.transform(ball_buffer, self.odom_frame, timeout=Duration(seconds=1.0))
            self.ball_map = self._blackboard.tf_buffer.transform(ball_buffer, self.map_frame, timeout=Duration(seconds=1.0))
            # Set timestamps to zero to get the newest transform when this is transformed later
            self.ball_odom.header.stamp = Time(seconds=0, nanoseconds=0, clock_type=ClockType.ROS_TIME).to_msg()
            self.ball_map.header.stamp = Time(seconds=0, nanoseconds=0, clock_type=ClockType.ROS_TIME).to_msg()
            self.ball_seen_time = Time.from_msg(msg.header.stamp)
            self.ball_publisher.publish(self.ball)
            self.ball_seen = True

        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self._blackboard.node.get_logger().warn(str(e))

    def recent_ball_twist_available(self) ->  bool:
        if self.ball_twist_map is None:
            return False
        return self._blackboard.node.get_clock().now() - self.ball_twist_map.header.stamp < self.ball_twist_lost_time

    def ball_twist_callback(self, msg: TwistWithCovarianceStamped):
        x_sdev = msg.twist.covariance[0]  # position 0,0 in a 6x6-matrix
        y_sdev = msg.twist.covariance[7]  # position 1,1 in a 6x6-matrix
        if x_sdev > self.ball_twist_precision_threshold['x_sdev'] or \
                y_sdev > self.ball_twist_precision_threshold['y_sdev']:
            return
        if msg.header.frame_id != self.map_frame:
            try:
                # point (0,0,0)
                point_a = PointStamped()
                point_a.header = msg.header
                # linear velocity vector
                point_b = PointStamped()
                point_b.header = msg.header
                point_b.point.x = msg.twist.twist.linear.x
                point_b.point.y = msg.twist.twist.linear.y
                point_b.point.z = msg.twist.twist.linear.z
                # transform start and endpoint of velocity vector
                point_a = self._blackboard.tf_buffer.transform(point_a, self.map_frame, timeout=Duration(seconds=1.0))
                point_b = self._blackboard.tf_buffer.transform(point_b, self.map_frame, timeout=Duration(seconds=1.0))
                # build new twist using transform vector
                self.ball_twist_map = TwistStamped(header=msg.header)
                self.ball_twist_map.header.frame_id = self.map_frame
                self.ball_twist_map.twist.linear.x = point_b.point.x - point_a.point.x
                self.ball_twist_map.twist.linear.y = point_b.point.y - point_a.point.y
                self.ball_twist_map.twist.linear.z = point_b.point.z - point_a.point.z
            except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                self._blackboard.node.get_logger().warn(str(e))
        else:
            self.ball_twist_map = TwistStamped(header=msg.header, twist=msg.twist.twist)
        if self.ball_twist_map is not None:
            self.ball_twist_publisher.publish(self.ball_twist_map)

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
            self.ball_seen_time = Time(seconds=0, nanoseconds=0, clock_type=ClockType.ROS_TIME)
            self.ball = PointStamped()

        if team:  # Forget team ball
            self.ball_seen_time_teammate = Time(seconds=0, nanoseconds=0, clock_type=ClockType.ROS_TIME)
            self.ball_teammate = PointStamped()

        if reset_ball_filter:  # Reset the ball filter
            result: Trigger.Response = self.reset_ball_filter.call(Trigger.Request())
            if result.success:
                self._blackboard.node.get_logger().info(f"Received message from ball filter: '{result.message}'")
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

    def get_current_position(self) -> Tuple[float, float, float]:
        """
        Returns the current position as determined by the localization
        :returns x,y,theta
        """
        transform = self.get_current_position_transform()
        if transform is None:
            return None
        orientation = transform.transform.rotation
        theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        return transform.transform.translation.x, transform.transform.translation.y, theta

    def get_current_position_pose_stamped(self) -> PoseStamped:
        """
        Returns the current position as determined by the localization as a PoseStamped
        """
        transform = self.get_current_position_transform()
        if transform is None:
            return None
        ps = PoseStamped()
        ps.header = transform.header
        ps.pose.position.x = transform.transform.translation.x
        ps.pose.position.y = transform.transform.translation.y
        ps.pose.position.z = transform.transform.translation.z
        ps.pose.orientation = transform.transform.rotation
        return ps

    def get_current_position_transform(self) -> TransformStamped:
        """
        Returns the current position as determined by the localization as a TransformStamped
        """
        try:
            # get the most recent transform
            transform = self._blackboard.tf_buffer.lookup_transform(self.map_frame, self.base_footprint_frame,
                                                        Time(seconds=0, nanoseconds=0, clock_type=ClockType.ROS_TIME))
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            self._blackboard.node.get_logger().warn(str(e))
            return None
        return transform

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
        return precision[0] < self.localization_precision_threshold['x_sdev'] and \
               precision[1] < self.localization_precision_threshold['y_sdev'] and \
               precision[2] < self.localization_precision_threshold['theta_sdev']

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
            t = Time(seconds=0, nanoseconds=0, clock_type=ClockType.ROS_TIME)
        return self._blackboard.tf_buffer.can_transform(self.base_footprint_frame, self.map_frame, t)

    ##########
    # Common #
    ##########

    def get_uv_from_xy(self, x, y) -> Tuple[float, float]:
        """ Returns the relativ positions of the robot to this absolute position"""
        current_position = self.get_current_position()
        x2 = x - current_position[0]
        y2 = y - current_position[1]
        theta = -1 * current_position[2]
        u = math.cos(theta) * x2 + math.sin(theta) * y2
        v = math.cos(theta) * y2 - math.sin(theta) * x2
        return u, v

    def get_xy_from_uv(self, u, v):
        """ Returns the absolute position from the given relative position to the robot"""
        pos_x, pos_y, theta = self.get_current_position()
        angle = math.atan2(v, u) + theta
        hypotenuse = math.hypot(u, v)
        return pos_x + math.sin(angle) * hypotenuse, pos_y + math.cos(angle) * hypotenuse

    def get_distance_to_xy(self, x, y):
        """ Returns distance from robot to given position """
        u, v = self.get_uv_from_xy(x, y)
        dist = math.hypot(u, v)
        return dist



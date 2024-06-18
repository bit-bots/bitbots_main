#! /usr/bin/env python3
from typing import Optional, Tuple

import numpy as np
import rclpy
import tf2_ros as tf2
from bitbots_tf_buffer import Buffer
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from ros2_numpy import msgify, numpify
from soccer_vision_3d_msgs.msg import Ball, BallArray
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_geometry_msgs import PointStamped

from bitbots_ball_filter.ball_filter_parameters import bitbots_ball_filter as parameters


class BallFilter(Node):
    ball_state_position: np.ndarray
    ball_state_covariance: np.ndarray
    config: parameters.Params

    def __init__(self) -> None:
        """
        creates filter and subscribes to messages which are needed
        """
        super().__init__("ball_filter")
        self.logger = self.get_logger()
        self.tf_buffer = Buffer(self, Duration(seconds=2))
        # Setup dynamic reconfigure config
        self.param_listener = parameters.ParamListener(self)

        # Initialize parameters
        self.update_params()
        self.logger.info(f"Using frame '{self.config.filter_frame}' for ball filtering")

        # Initialize state
        self.reset_ball()

        # publishes positions of ball
        self.ball_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.config.ball_position_publish_topic, 1
        )

        # Create callback group
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Setup subscriber
        self.subscriber = self.create_subscription(
            BallArray,
            self.config.ball_subscribe_topic,
            self.ball_callback,
            2,
            callback_group=self.callback_group,
        )

        self.reset_service = self.create_service(
            Trigger,
            self.config.ball_filter_reset_service_name,
            self.reset_filter_cb,
            callback_group=self.callback_group,
        )

        self.filter_timer = self.create_timer(
            self.filter_time_step, self.filter_step, callback_group=self.callback_group
        )

    def reset_ball(self) -> None:
        self.ball_state_position = np.zeros(3)
        self.ball_state_covariance = np.eye(3) * 1000

    def reset_filter_cb(self, req, response) -> Tuple[bool, str]:
        self.logger.info("Resetting bitbots ball filter...")
        self.reset_ball()
        response.success = True
        return response

    def get_position_numpy(self) -> np.ndarray:
        return np.array([*self.get_position_tuple()])

    def ball_callback(self, msg: BallArray) -> None:
        if msg.balls:  # Balls exist
            # Ignore balls that are too far away (false positives)
            # The ignore distance is calculated using the filter's covariance and a factor
            # This way false positives are ignored if we already have a good estimate
            ignore_threshold_x, ignore_threshold_y, _ = (
                np.sqrt(np.diag(self.ball_state_covariance)) * self.config.ignore_measurement_threshold
            )

            # Filter out balls that are too far away from the filter's estimate
            filtered_balls: list[
                tuple[Ball, PointStamped, float]
            ] = []  # Store, original ball in base_footprint frame, transformed ball in filter frame , distance to filter estimate
            ball: Ball
            for ball in msg.balls:
                ball_transform = self._get_transform(msg.header, ball.center)
                if ball_transform:
                    diff = numpify(ball_transform.point) - self.get_position_numpy()
                    if abs(diff[0]) < ignore_threshold_x and abs(diff[0]) < ignore_threshold_y:
                        filtered_balls.append((ball, ball_transform, np.linalg.norm(diff)))

            # Select the ball with closest distance to the filter estimate
            # Return if no ball was found
            ball_msg, ball_measurement_map, _ = min(filtered_balls, key=lambda x: x[2], default=(None, None, 0))
            if ball_measurement_map is None:
                return

            # Estimate the covariance of the measurement
            # Calculate the distance from the robot to the ball
            distance = np.linalg.norm(numpify(ball_msg.center))
            covariance = np.eye(3) * (
                self.config.measurement_certainty + (distance**2) * self.config.noise_increment_factor
            )
            covariance[2, 2] = 0.0  # Ignore z-axis

            # Store the ball measurement
            self.ball_state_position = numpify(ball_measurement_map.point)
            self.ball_state_covariance = covariance

    def _get_transform(
        self, header: Header, point: Point, frame: Optional[str] = None, timeout: float = 0.3
    ) -> Optional[PointStamped]:
        """
        Transforms a point to the filter frame
        """

        if frame is None:
            frame = self.config.filter_frame

        point_stamped = PointStamped()
        point_stamped.header = header
        point_stamped.point = point

        try:
            return self.tf_buffer.transform(point_stamped, frame, timeout=Duration(nanoseconds=int(timeout * (10**9))))
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.logger.warning(str(e))

    def update_params(self) -> None:
        """
        Updates parameters from dynamic reconfigure
        """
        self.config = self.param_listener.get_params()
        self.filter_time_step = 1.0 / self.config.filter_rate
        self.filter_reset_duration = Duration(seconds=self.config.filter_reset_time)

    def filter_step(self) -> None:
        """
        When ball has been assigned a value and filter has been initialized:
        State will be updated according to filter.
        If filter has not been initialized, then that will be done first.

        If there is no data for ball, a prediction will still be made and published
        Process noise is taken into account
        """
        # check whether parameters have changed
        if self.param_listener.is_old(self.config):
            self.param_listener.refresh_dynamic_parameters()
            self.update_params()

        # Increase covariance
        self.ball_state_covariance += np.eye(2) * self.config.process_noise_variance

        # Pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header(
            stamp=Time.to_msg(self.get_clock().now()),
            frame_id=self.config.filter_frame,
        )
        pose_msg.pose.pose.position = msgify(Point, self.ball_state_position)
        pose_msg.pose.covariance = np.zeros((6, 6))
        pose_msg.pose.covariance[:3, :3] = self.ball_state_covariance
        pose_msg.pose.pose.orientation.w = 1.0
        self.ball_pose_publisher.publish(pose_msg)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = BallFilter()
    # Number of executor threads is the number of MutiallyExclusiveCallbackGroups + 1 thread for the executor
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()

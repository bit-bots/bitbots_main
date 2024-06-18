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
from sensor_msgs.msg import CameraInfo
from rclpy.time import Time
from ros2_numpy import msgify, numpify
from soccer_vision_3d_msgs.msg import Ball, BallArray
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_geometry_msgs import PointStamped, PoseStamped

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

        self.camera_info: Optional[CameraInfo] = None

        # Initialize state
        self.reset_ball()

        # publishes positions of ball
        self.ball_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.config.ball_position_publish_topic, 1
        )

        # Create callback group
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # setup subscriber
        self.ball_subscriber = self.create_subscription(
            BallArray,
            self.config.ball_subscribe_topic,
            self.ball_callback,
            2,
            callback_group=self.callback_group,
        )

        self.camera_info_subscriber = self.create_subscription(
            CameraInfo, self.config.camera_info_subscribe_topic, self.camera_info_callback, 1
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

    def camera_info_callback(self, msg: CameraInfo):
        """handles incoming ball messages"""
        self.camera_info = msg

    def ball_callback(self, msg: BallArray) -> None:
        # Keep track if we have updated the measurement
        # We might not have a measurement if the ball is not visible
        # We also filter out balls that are too far away from the filter's estimate
        ball_measurement_updated: bool = False

        # Do filtering, transform, ... if we have a ball
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
                    diff = numpify(ball_transform.point) - self.ball_state_position
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
            ball_measurement_updated = True

        # If we did not get a ball measurement, we can check if we should have seen the ball
        # And increase the covariance if we did not see the ball
        if not ball_measurement_updated and self.is_estimate_in_fov(msg.header):
            self.ball_state_covariance += np.eye(3) * self.config.negative_observation_value

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

    def is_estimate_in_fov(self, header: Header) -> bool:
        """
        Calculates if a ball should be currently visible
        """
        # Check if we got a camera info to do this stuff
        if self.camera_info is None:
            self.logger.info("No camera info received. Not checking if the ball is currently visible.")
            return False
        
        # Build a pose
        ball_pose = PoseStamped()
        ball_pose.header.frame_id = self.config.filter_frame
        ball_pose.header.stamp = header.stamp
        ball_pose.pose.position = msgify(Point, self.ball_state_position)

        # Transform to camera frame
        try:
            ball_in_camera_optical_frame = self.tf_buffer.transform(
                ball_pose, self.camera_info.header.frame_id, timeout=Duration(nanoseconds=0.5 * (10**9))
            )
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.logger.warning(str(e))
            return False

        # Check if the ball is in front of the camera
        if ball_in_camera_optical_frame.pose.position.z >= 0:
            # Quick math to get the pixels
            p = numpify(ball_in_camera_optical_frame.pose.position)
            k = np.reshape(self.camera_info.k, (3, 3))
            p_pixel = np.matmul(k, p)
            p_pixel = p_pixel * (1 / p_pixel[2])
            # Make sure that the transformed pixel is inside the resolution and positive.
            # TODO border
            if 0 < p_pixel[0] <= self.camera_info.width and 0 < p_pixel[1] <= self.camera_info.height:
                return True
        return False

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
        self.ball_state_covariance += np.eye(3) * self.config.process_noise_variance

        # Pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header(
            stamp=Time.to_msg(self.get_clock().now()),
            frame_id=self.config.filter_frame,
        )
        pose_msg.pose.pose.position = msgify(Point, self.ball_state_position)
        covariance = np.zeros((6, 6))
        covariance[:3, :3] = self.ball_state_covariance
        pose_msg.pose.covariance = covariance.flatten()
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

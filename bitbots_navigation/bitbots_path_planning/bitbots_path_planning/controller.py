import math
from typing import Optional

import tf2_ros as tf2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time
from ros2_numpy import numpify
from tf2_geometry_msgs import PointStamped, Pose, PoseStamped
from tf_transformations import euler_from_quaternion


class Controller:
    """
    A simple follow the carrot controller which controls the robots command velocity to stay on a given path.
    """

    def __init__(self, node: Node, buffer: tf2.Buffer) -> None:
        self.node = node
        self.buffer = buffer

        # Last command velocity
        self.last_cmd_vel = Twist()

        # Last update time
        self.last_update_time: Optional[Time] = None

        # Accumulator for the angular error
        self.angular_error_accumulator = 0

    def step(self, path: Path) -> tuple[Twist, PointStamped]:
        """
        Calculates a command velocity based on a given path
        """
        # Starting point for our goal velocity calculation
        cmd_vel = Twist()

        # Create an default pose in the origin of our base footprint
        pose_geometry_msgs = PoseStamped()
        pose_geometry_msgs.header.frame_id = self.node.config.base_footprint_frame

        # We get our pose in respect to the map frame (also frame of the path message)
        # by transforming the pose above into this frame
        try:
            current_pose: Pose = self.buffer.transform(pose_geometry_msgs, path.header.frame_id).pose
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.node.get_logger().warn("Failed to perform controller step: " + str(e))
            return cmd_vel

        # Select pose for the carrot (goal_pose for this controller) and the end pose of the global plan
        # End conditions with an empty or shorter than carrot distance path need to be considered
        if len(path.poses) > 0:
            end_pose: Pose = path.poses[-1].pose
            if len(path.poses) > self.node.config.controller.carrot_distance:
                goal_pose: Pose = path.poses[self.node.config.controller.carrot_distance].pose
            else:
                goal_pose: Pose = end_pose
        else:
            return cmd_vel

        # Calculate the intended walk angle based on the carrot and our position
        walk_angle = math.atan2(
            goal_pose.position.y - current_pose.position.y, goal_pose.position.x - current_pose.position.x
        )

        # Calculate the heading angle from our current position to the final position of the global plan
        final_walk_angle = math.atan2(
            end_pose.position.y - current_pose.position.y, end_pose.position.x - current_pose.position.x
        )

        if len(path.poses) < 3:
            # Calculate the distance from our current position to the final position of the global plan
            distance = math.hypot(
                end_pose.position.x - current_pose.position.x, end_pose.position.y - current_pose.position.y
            )
        else:
            # Calculate path length
            distance = 0
            a_position = current_pose.position
            pose: PoseStamped
            for pose in path.poses:
                b_postion = pose.pose.position
                distance += math.hypot(b_postion.x - a_position.x, b_postion.y - a_position.y)
                a_position = b_postion
            distance += math.hypot(end_pose.position.x - a_position.x, end_pose.position.y - a_position.y)

        # Calculate the translational walk velocity.
        # It considers the distance and breaks if we are close to the final position of the global plan
        walk_vel = distance * self.node.config.controller.translation_slow_down_factor

        # Check if we are so close to the final position of the global plan that we want to align us with
        # its orientation and not the heading towards its position
        diff = 0
        if distance > self.node.config.controller.orient_to_goal_distance:
            # Calculate the difference between our current heading and the heading towards the final position of
            # the global plan
            diff = final_walk_angle - self._get_yaw(current_pose)
        else:
            # Calculate the difference between our current heading and the heading of the final position of
            # the global plan
            diff = self._get_yaw(end_pose) - self._get_yaw(current_pose)

        # Get the min angle of the difference
        min_angle = math.remainder(diff, math.tau)

        # Accumulate the angular error (integral)
        self.angular_error_accumulator += min_angle

        # Define the target rotation velocity (based on p and i of the error)
        target_rot_vel = (
            self.node.config.controller.rotation_slow_down_factor * min_angle
            + self.node.config.controller.rotation_i_factor * self.angular_error_accumulator
        )

        # Clamp rotational velocity to max values (both sides)
        rot_goal_vel = min(
            max(target_rot_vel, -self.node.config.controller.max_rotation_vel),
            self.node.config.controller.max_rotation_vel,
        )

        # Calculate the heading of our linear velocity vector in the local frame of the robot
        local_heading = walk_angle - self._get_yaw(current_pose)

        # Convert the heading of the robot to a unit vector
        local_heading_vector_x = math.cos(local_heading)
        local_heading_vector_y = math.sin(local_heading)

        # Returns interpolates the x and y maximum velocity linearly based on our heading
        # See https://github.com/bit-bots/bitbots_main/issues/366#issuecomment-2161460917
        def interpolate_max_vel(x_limit, y_limit, target_x, target_y):
            n = (x_limit * abs(target_y)) + y_limit * target_x
            intersection_x = (x_limit * y_limit * target_x) / n
            intersection_y = (x_limit * y_limit * abs(target_y)) / n
            return math.hypot(intersection_x, intersection_y)

        # Limit the x and y velocity with the heading specific maximum velocity
        walk_vel = min(
            walk_vel,
            # Calc the max velocity for the current heading
            interpolate_max_vel(
                # Use different maximum values for forward and backwards x movement
                self.node.config.controller.max_vel_x
                if local_heading_vector_x > 0
                else self.node.config.controller.min_vel_x,
                self.node.config.controller.max_vel_y,
                local_heading_vector_x,
                local_heading_vector_y,
            ),
        )

        # Calculate the x and y components of our linear velocity based on the desired heading and the
        # desired translational velocity.
        cmd_vel.linear.x = local_heading_vector_x * walk_vel
        cmd_vel.linear.y = local_heading_vector_y * walk_vel

        # Apply the desired rotational velocity
        cmd_vel.angular.z = rot_goal_vel

        # Filter the command velocity to avoid sudden changes
        current_time = self.node.get_clock().now()

        # Don't apply the filter on the first update
        if self.last_update_time is not None:
            # Calculate the time since the last update
            delta_time = current_time - self.last_update_time

            # Calculate the exponential decay smoothing factor alpha from the time constant tau
            # Instead of defining alpha directly, we use tau to derive it in a time-step independent way
            exponent_in_s = -delta_time.nanoseconds / (self.node.config.controller.smoothing_tau * 1e9)
            alpha = 1 - math.exp(exponent_in_s)

            # Apply the exponential moving average filter
            cmd_vel.linear.x = alpha * cmd_vel.linear.x + (1 - alpha) * self.last_cmd_vel.linear.x
            cmd_vel.linear.y = alpha * cmd_vel.linear.y + (1 - alpha) * self.last_cmd_vel.linear.y
            cmd_vel.angular.z = alpha * cmd_vel.angular.z + (1 - alpha) * self.last_cmd_vel.angular.z

        # Store the current time for the next update
        self.last_update_time = current_time

        # Store the last command velocity
        self.last_cmd_vel = cmd_vel

        # Create a carrot message for visualization
        carrot_point = PointStamped()
        carrot_point.header.frame_id = path.header.frame_id
        carrot_point.point = goal_pose.position

        return cmd_vel, carrot_point

    def _get_yaw(self, pose: Pose) -> float:
        """
        Returns the yaw angle of a given pose
        """
        return euler_from_quaternion(numpify(pose.orientation))[2]

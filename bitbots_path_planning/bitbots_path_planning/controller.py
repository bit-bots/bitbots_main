import math

import tf2_ros as tf2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from ros2_numpy import numpify
from tf2_geometry_msgs import Pose, PoseStamped
from tf_transformations import euler_from_quaternion


class Controller:
    """
    A simple follow the carrot controller which controls the robots command velocity to stay on a given path.
    """
    def __init__(self, node: Node, buffer: tf2.Buffer) -> None:
        self.node = node
        self.buffer = buffer

        self.config_base_footprint_frame: str = self.node.get_parameter("base_footprint_frame").value
        self.config_carrot_distance: int = self.node.declare_parameter('controller.carrot_distance', 20).value
        self.config_max_rotation_vel: float = self.node.declare_parameter('controller.max_rotation_vel', 0.4).value
        self.config_max_vel_x: float = self.node.declare_parameter('controller.max_vel_x', 0.25).value
        self.config_max_vel_y: float = self.node.declare_parameter('controller.max_vel_y', 0.08).value
        self.config_min_vel_x: float = self.node.declare_parameter('controller.min_vel_x', -0.2).value
        self.config_orient_to_goal_distance: float = self.node.declare_parameter('controller.orient_to_goal_distance', 1.0).value
        self.config_position_accuracy: float = self.node.declare_parameter('controller.position_accuracy', 0.05).value
        self.config_rotation_accuracy: float = self.node.declare_parameter('controller.rotation_accuracy', 0.1).value
        self.config_rotation_slow_down_factor: float = self.node.declare_parameter('controller.rotation_slow_down_factor', 0.3).value
        self.config_smoothing_k: float = self.node.declare_parameter('controller.smoothing_k', 0.4).value
        self.config_translation_slow_down_factor: float = self.node.declare_parameter('controller.translation_slow_down_factor', 0.5).value

    def step(self, path: Path) -> Twist:
        """
        Calculates a command velocity based on a given path
        """
        # Starting point for our goal velocity calculation
        cmd_vel = Twist()

        # Create an default pose in the origin of our base footprint
        pose_geometry_msgs = PoseStamped()
        pose_geometry_msgs.header.frame_id = self.config_base_footprint_frame

        # We get our pose in respect to the map frame (also frame of the path message)
        # by transforming the pose above into this frame
        try:
            current_pose: Pose = self.buffer.transform(pose_geometry_msgs, path.header.frame_id).pose
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                self.node.get_logger().warn('Failed to perform controller step: ' + str(e))
                return cmd_vel

        # Select pose for the carrot (goal_pose for this controller) and the end pose of the global plan
        # End conditions with an empty or shorter than carrot distance path need to be considered
        if len(path.poses) > 0:
            end_pose: Pose = path.poses[-1].pose
            if len(path.poses) > self.config_carrot_distance:
                goal_pose: Pose = path.poses[self.config_carrot_distance].pose
            else:
                goal_pose: Pose = end_pose
        else:
            return cmd_vel

        # Calculate the intended walk angle based on the carrot and our position
        walk_angle = math.atan2(
            goal_pose.position.y - current_pose.position.y,
            goal_pose.position.x - current_pose.position.x)

        # Calculate the heading angle from our current position to the final position of the global plan
        final_walk_angle = math.atan2(
            end_pose.position.y - current_pose.position.y,
            end_pose.position.x - current_pose.position.x)

        # Calculate the distance from our current position to the final position of the global plan
        distance = math.hypot(
            end_pose.position.x - current_pose.position.x,
            end_pose.position.y - current_pose.position.y)

        # Calculate the translational walk velocity. It considers the distance and breaks if we are close to the final position of the global plan
        walk_vel = min(distance * self.config_translation_slow_down_factor, self.config_max_vel_x)

        # Check if we are so close to the final position of the global plan that we want to align us with its orientation and not the heading towards its position
        diff = 0
        if distance > self.config_orient_to_goal_distance:
            # Calculate the difference between our current heading and the heading towards the final position of the global plan
            diff = final_walk_angle - self._get_yaw(current_pose)
        else:
            # Calculate the difference between our current heading and the heading of the final position of the global plan
            diff = self._get_yaw(end_pose) - self._get_yaw(current_pose)

        # Get the min angle of the difference
        min_angle = math.remainder(diff, math.tau)
        # Calculate our desired rotation velocity based on the angle difference and our max velocity
        rot_goal_vel = min(max(self.config_rotation_slow_down_factor * min_angle, -self.config_max_rotation_vel), self.config_max_rotation_vel)

        # Calculate the x and y components of our linear velocity based on the desired heading and the desired translational velocity.
        cmd_vel.linear.x = math.cos(walk_angle - self._get_yaw(current_pose)) * walk_vel
        cmd_vel.linear.y = math.sin(walk_angle - self._get_yaw(current_pose)) * walk_vel

        # Scale command accordingly if a limit is acceded
        if cmd_vel.linear.x > self.config_max_vel_x:
            self.node.get_logger().debug("X LIMIT reached: %f > %f, with y %f".format(cmd_vel.linear.y, self.config_max_vel_x, cmd_vel.linear.y));
            cmd_vel.linear.y *= self.config_max_vel_x / cmd_vel.linear.x
            cmd_vel.linear.x = self.config_max_vel_x
            self.node.get_logger().debug("X LIMIT set y %f".format(cmd_vel.linear.y))

        if cmd_vel.linear.x < self.config_min_vel_x:
            self.node.get_logger().debug("X LIMIT reached: %f < %f, with y %f".format(cmd_vel.linear.x, self.config_min_vel_x, cmd_vel.linear.y))
            cmd_vel.linear.y *= self.config_min_vel_x / cmd_vel.linear.x
            cmd_vel.linear.x = self.config_min_vel_x
            self.node.get_logger().debug("X LIMIT set y %f".format(cmd_vel.linear.y))

        max_y = self.config_max_vel_y

        if abs(cmd_vel.linear.y) > max_y:
            self.node.get_logger().debug("Y LIMIT reached: %f > %f, with x %f".format(cmd_vel.linear.y, max_y, cmd_vel.linear.x))
            cmd_vel.linear.x *= max_y / abs(cmd_vel.linear.y)
            cmd_vel.linear.y *= max_y / abs(cmd_vel.linear.y)
            self.node.get_logger().debug("Y LIMIT set x %f".format(cmd_vel.linear.x))

        # Apply the desired rotational velocity
        cmd_vel.angular.z = rot_goal_vel

        return cmd_vel

    def _get_yaw(self, pose: Pose) -> float:
        """
        Returns the yaw angle of a given pose
        """
        return euler_from_quaternion(numpify(pose.orientation))[2]

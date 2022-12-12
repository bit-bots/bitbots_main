import math

import tf2_ros as tf2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from ros2_numpy import numpify
from tf2_geometry_msgs import Pose, PoseStamped
from tf_transformations import euler_from_quaternion


class Controller:
    def __init__(self, node: Node, buffer: tf2.Buffer) -> None:
        self.node = node
        self.buffer = buffer

        self.config_carrot_distance = 20
        self.config_rotation_accuracy = 0.1
        self.config_position_accuracy = 0.05
        self.config_max_rotation_vel = 0.4
        self.config_max_vel_x = 0.25
        self.config_min_vel_x = -0.2
        self.config_max_vel_y = 0.08
        self.config_smoothing_k = 0.4
        self.config_rotation_slow_down_factor = 0.3
        self.config_translation_slow_down_factor = 0.5
        self.config_orient_to_goal_distance = 1.0

    def _get_yaw(self, pose) -> float:
        return euler_from_quaternion(numpify(pose))[2]

    def step(self, path: Path) -> Twist:
        pose_geometry_msgs = PoseStamped()
        pose_geometry_msgs.header.frame_id = "base_footprint"
        current_pose: Pose = self.buffer.transform(pose_geometry_msgs, path.header.frame_id).pose

        cmd_vel = Twist()

        if len(path.poses) > 0:
            end_pose: Pose = path.poses[-1].pose
            if len(path.poses) > self.config_carrot_distance:
                goal_pose: Pose = path.poses[self.config_carrot_distance].pose
            else:
                goal_pose: Pose = end_pose
        else:
            return cmd_vel

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
            diff = final_walk_angle - self._get_yaw(current_pose.orientation)
        else:
            # Calculate the difference between our current heading and the heading of the final position of the global plan
            diff = self._get_yaw(end_pose.orientation) - self._get_yaw(current_pose.orientation)

        # Get the min angle of the difference
        min_angle = math.remainder(diff, math.tau)
        # Calculate our desired rotation velocity based on the angle difference and our max velocity
        rot_goal_vel = min(max(self.config_rotation_slow_down_factor * min_angle, -self.config_max_rotation_vel), self.config_max_rotation_vel)

        # Calculate the x and y components of our linear velocity based on the desired heading and the desired translational velocity.
        cmd_vel.linear.x = math.cos(walk_angle - self._get_yaw(current_pose.orientation)) * walk_vel
        cmd_vel.linear.y = math.sin(walk_angle - self._get_yaw(current_pose.orientation)) * walk_vel

        # Scale command accordingly if a limit is acceded
        if cmd_vel.linear.x > self.config_max_vel_x:
            self.node.get_logger().debug("X LIMIT reached: %f > %f, with y %f".format(cmd_vel.linear.y, self.config_max_vel_x, cmd_vel.linear.y));
            cmd_vel.linear.y *= self.config_max_vel_x / cmd_vel.linear.x
            cmd_vel.linear.x = self.config_max_vel_x
            self.node.get_logger().debug("X LIMIT set y %f".format(cmd_vel.linear.y))

        if cmd_vel.linear.x < self.config_min_vel_x:
            self.node.get_logger().debug("X LIMIT reached: %f < %f, with y %f".format(cmd_vel.linear.x, self.config_min_vel_x, cmd_vel.linear.y))
            cmd_vel.linear.y *= self.config_min_vel_x / cmd_vel.linear.x
            cmd_vel.linear.x =self. config_min_vel_x
            self.node.get_logger().debug("X LIMIT set y %f".format(cmd_vel.linear.y))

        max_y = self.config_max_vel_y

        if abs(cmd_vel.linear.y) > max_y:
            self.node.get_logger().debug("Y LIMIT reached: %f > %f, with x %f".format(cmd_vel.linear.y, max_y, cmd_vel.linear.x))
            cmd_vel.linear.x *= max_y / abs(cmd_vel.linear.y)
            cmd_vel.linear.y *= max_y / abs(cmd_vel.linear.y)
            self.node.get_logger().debug("Y LIMIT set x %f".format(cmd_vel.linear.x))

        # Complementary Filter for smooting the outputs
        #cmd_vel.linear.x = cmd_vel.linear.x * self.config_smoothing_k + robot_vel.pose.position.x * (1.0 - self.config_smoothing_k)
        #cmd_vel.linear.y = cmd_vel.linear.y *self. config_smoothing_k + robot_vel.pose.position.y * (1.0 - self.config_smoothing_k)

        # Apply the desired rotational velocity
        cmd_vel.angular.z = rot_goal_vel

        return cmd_vel

        #self.node.get_logger().debug("End vel %f, %f\n".format(math.hypot(cmd_vel.linear.x, cmd_vel.linear.y), current_vel_))
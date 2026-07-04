import json
import math

import numpy as np
from rclpy.duration import Duration

from bitbots_animation_server.animation import parse
from bitbots_animation_server.resource_manager import ResourceManager


class WalkreadyTransition:
    """Linearly interpolates the robot from its current pose into the walkready pose.

    After the RL kick policy stops publishing, the robot would otherwise hold the
    policy's last commanded pose. This helper reads the walkready animation from the
    ``<robot_type>_animations`` package (via the bitbots_animation_server resource
    manager / parser), takes its final keyframe as the goal pose and blends the
    controlled joints from their measured positions to that goal with a straight
    linear interpolation in joint space.

    The interpolated commands are published on the kick node's ``kick_motor_goals``
    publisher at the control rate, which keeps the HCM in its KICKING state (it stays
    there while fresh kick goals arrive) so the commands are forwarded to the servos
    for the whole transition.
    """

    def __init__(self, node, joint_handler, joint_command_pub):
        self._node = node
        self._joint_handler = joint_handler
        self._joint_command_pub = joint_command_pub

        self._control_dt = float(node.get_parameter("phase.control_dt").value)
        self._duration = float(node.get_parameter("command.walkready_transition_duration").value)
        robot_type = str(node.get_parameter("robot_type").value)
        animation_name = str(node.get_parameter("command.walkready_animation").value)

        self._target = self._load_target(robot_type, animation_name)

    def _load_target(self, robot_type: str, animation_name: str) -> np.ndarray:
        """Load the walkready goal pose (radians) for the published joints, in publish order."""
        resource_manager = ResourceManager(robot_type)
        path = resource_manager.find_animation(animation_name)
        with open(path) as f:
            animation = parse(json.load(f))

        if not animation.keyframes:
            raise ValueError(f"Walkready animation '{animation_name}' has no keyframes")

        # The walkready animation ends in the walk-ready stance; use its final keyframe.
        goals_deg = animation.keyframes[-1].goals

        published = self._joint_handler.published_joint_names
        missing = [name for name in published if name not in goals_deg]
        if missing:
            raise ValueError(
                f"Walkready animation '{animation_name}' is missing goals for published joints: {missing}"
            )

        # Animation goals are stored in degrees; the JointCommand expects radians.
        return np.array([math.radians(goals_deg[name]) for name in published], dtype=np.float32)

    def run(self, goal_handle=None, feedback=None) -> bool:
        """Interpolate from the current pose to walkready, publishing at the control rate.

        Blocks for ``walkready_transition_duration`` seconds (called from the kick
        action's own thread). Returns ``True`` when the transition completed and
        ``False`` if it was cancelled part-way through.
        """
        published = self._joint_handler.published_joint_names
        start = self._joint_handler.get_measured_positions(published)

        # A zero/negative duration means "no smoothing": command the goal once.
        steps = max(1, int(round(self._duration / self._control_dt)))

        self._node.get_logger().info(f"Transitioning to walkready over {self._duration:.2f} s ({steps} steps).")

        for step in range(1, steps + 1):
            if goal_handle is not None and goal_handle.is_cancel_requested:
                self._node.get_logger().info("Walkready transition cancelled.")
                return False

            alpha = step / steps
            positions = (1.0 - alpha) * start + alpha * self._target
            self._joint_command_pub.publish(self._joint_handler.get_absolute_joint_command(positions))

            if feedback is not None and goal_handle is not None:
                feedback.time_remaining = max(0.0, (steps - step) * self._control_dt)
                goal_handle.publish_feedback(feedback)

            self._node.get_clock().sleep_for(Duration(seconds=self._control_dt))

        return True

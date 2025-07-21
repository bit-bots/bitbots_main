import math
from enum import Enum
from typing import Optional

import numpy as np
from bitbots_utils.transforms import quat_from_yaw
from bitbots_utils.utils import get_parameters_from_other_node
from geometry_msgs.msg import Point, PoseStamped, Twist
from ros2_numpy import numpify
from std_msgs.msg import Bool, Empty
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

from bitbots_blackboard.capsules import AbstractBlackboardCapsule


# Type of pathfinding goal relative to the ball
class BallGoalType(str, Enum):
    GRADIENT = "gradient"
    MAP = "map"
    CLOSE = "close"


class PathfindingCapsule(AbstractBlackboardCapsule):
    """Capsule for pathfinding related functions."""

    def __init__(self, node, blackboard):
        super().__init__(node, blackboard)
        self.position_threshold: float = self._node.get_parameter("pathfinding_position_threshold").value
        self.orientation_threshold: float = self._node.get_parameter("pathfinding_orientation_threshold").value

        self.direct_cmd_vel_pub = self._node.create_publisher(Twist, "cmd_vel", 1)
        self.pathfinding_pub = self._node.create_publisher(PoseStamped, "goal_pose", 1)
        self.pathfinding_cancel_pub = self._node.create_publisher(Empty, "pathfinding/cancel", 1)
        self.ball_obstacle_active_pub = self._node.create_publisher(Bool, "ball_obstacle_active", 1)
        self.approach_marker_pub = self._node.create_publisher(Marker, "debug/approach_point", 10)
        self.goal: Optional[PoseStamped] = None
        self.avoid_ball: bool = True
        self.current_cmd_vel = Twist()
        self.orient_to_ball_distance: float = get_parameters_from_other_node(
            self._node, "bitbots_path_planning", ["controller.orient_to_goal_distance"]
        )["controller.orient_to_goal_distance"]

    def publish(self, msg: PoseStamped) -> None:
        """
        Sends a goal to the pathfinding.
        """
        self.goal = msg
        self.pathfinding_pub.publish(msg)

    def get_goal(self) -> Optional[PoseStamped]:
        """
        Returns the latest goal that was send to the pathfinding.
        """
        return self.goal

    def cancel_goal(self) -> None:
        """
        This function cancels the current goal of the pathfinding,
        which will stop sending cmd_vel messages to the walking.
        This does not stop the walking itself.
        """
        self.pathfinding_cancel_pub.publish(Empty())

    def cmd_vel_cb(self, msg: Twist) -> None:
        self.current_cmd_vel = msg

    def get_current_cmd_vel(self) -> Twist:
        """
        Returns the latest cmd_vel message that was send to the walking.

        This message can be from the behavior or from the pathfinding.
        """
        return self.current_cmd_vel

    def stop_walk(self) -> None:
        """
        This function stops the walking. It does not cancel the current goal of the
        pathfinding and the walking will start again if the pathfinding sends a new message.
        """
        # send special command to walking to stop it
        msg = Twist()
        msg.angular.x = -1.0
        # Cancel the path planning
        self.cancel_goal()
        # Publish the stop command
        self.direct_cmd_vel_pub.publish(msg)

    def calculate_time_to_ball(self) -> None:
        """
        Calculates the time to ball and saves it in the team data capsule.
        """
        # only send new request if previous request is finished or first update
        # also verify that the ball and the localization are reasonably recent/accurate
        if self._blackboard.world_model.ball_has_been_seen():
            ball_target = self.get_ball_goal(BallGoalType.MAP, self._blackboard.config["ball_approach_dist"])
            own_position = self._blackboard.world_model.get_current_position_pose_stamped()
            self._blackboard.team_data.own_time_to_ball = self.time_from_pose_to_pose(own_position, ball_target)
        else:
            # since we can not get a reasonable estimate, we are lost and set the time_to_ball to a very high value
            self._blackboard.team_data.own_time_to_ball = 9999.0

    def time_from_pose_to_pose(self, own_pose: PoseStamped, goal_pose: PoseStamped) -> float:
        """
        This function approximates the behavior of the pathfinding to calculate the time
        in seconds from one pose to another. It does not consider obstacles.
        """
        start_point = own_pose.pose.position
        end_point = goal_pose.pose.position
        path_length = np.linalg.norm(numpify(start_point)[:2] - numpify(end_point)[:2])
        # if the robot is close to the ball it does not turn to walk to it
        if path_length < self.orient_to_ball_distance:
            _, _, start_theta = self._blackboard.world_model.get_current_position()
            goal_theta = euler_from_quaternion(numpify(goal_pose.pose.orientation))[2]
            start_goal_theta_diff = (abs(start_theta - goal_theta) + math.tau / 2) % math.tau - math.tau / 2
            start_goal_theta_cost = (
                start_goal_theta_diff * self._blackboard.config["time_to_ball_cost_start_to_goal_angle"]
            )
            total_cost = path_length * self._blackboard.config["time_to_ball_cost_per_meter"] + start_goal_theta_cost
        else:
            # calculate how much we need to turn to start walking along the path
            _, _, start_theta = self._blackboard.world_model.get_current_position()
            path_theta = math.atan2(end_point.y - start_point.y, end_point.x - start_point.x)
            start_theta_diff = (abs(start_theta - path_theta) + math.tau / 2) % math.tau - math.tau / 2
            # calculate how much we need to turn to turn at the end of the path
            goal_theta = euler_from_quaternion(numpify(goal_pose.pose.orientation))[2]
            goal_theta_diff = (abs(goal_theta - path_theta) + math.tau / 2) % math.tau - math.tau / 2
            start_theta_cost = start_theta_diff * self._blackboard.config["time_to_ball_cost_start_angle"]
            goal_theta_cost = goal_theta_diff * self._blackboard.config["time_to_ball_cost_goal_angle"]
            total_cost = (
                path_length * self._blackboard.config["time_to_ball_cost_per_meter"]
                + start_theta_cost
                + goal_theta_cost
            )
        return total_cost

    def get_ball_goal(self, target: BallGoalType, distance: float, side_offset: float = 0.0) -> PoseStamped:
        """
        This function returns a goal pose relative to the ball.

        The following targets are available:
        - gradient: The goal pose chosen so the ball is 'distance' meters in the direction indicated by the gradient of the costmap.
        - map: The goal pose chosen so the ball is 'distance' meters in the direction of the opponent goal.
        - close: The goal is inside the ball with us facing the ball.
        """

        if BallGoalType.GRADIENT == target:
            ball_x, ball_y = self._blackboard.world_model.get_ball_position_xy()

            goal_angle = self._blackboard.costmap.get_gradient_direction_at_field_position(ball_x, ball_y)

            goal_x = ball_x - math.cos(goal_angle) * distance
            goal_y = ball_y - math.sin(goal_angle) * distance

            ball_point = (goal_x, goal_y, goal_angle, self._blackboard.map_frame)

        elif BallGoalType.MAP == target:
            goal_angle = self._blackboard.world_model.get_map_based_opp_goal_angle_from_ball()

            ball_x, ball_y = self._blackboard.world_model.get_ball_position_xy()

            # Play in any part of the opponents goal, not just the center
            if abs(ball_y) < self._blackboard.world_model.goal_width / 2:
                goal_angle = 0
            # Play in the opposite direction if the ball is near the opponent goal back line
            elif ball_x > self._blackboard.world_model.field_length / 2 - 0.2:
                goal_angle = math.pi + np.copysign(math.pi / 4, ball_y)

            # We don't want to walk into the ball, so we add an offset to stop before the ball
            approach_offset_x = math.cos(goal_angle) * distance
            approach_offset_y = math.sin(goal_angle) * distance

            # We also want to kick the ball with one foot instead of the center between the feet
            side_offset_x = math.cos(goal_angle - math.pi / 2) * side_offset
            side_offset_y = math.sin(goal_angle - math.pi / 2) * side_offset

            # Calculate the goal position (has nothing to do with the soccer goal)
            goal_x = ball_x - approach_offset_x + side_offset_x
            goal_y = ball_y - approach_offset_y + side_offset_y

            ball_point = (goal_x, goal_y, goal_angle, self._blackboard.map_frame)

        elif BallGoalType.CLOSE == target:
            ball_u, ball_v = self._blackboard.world_model.get_ball_position_uv()
            ball_v_from_kick_foot = ball_v + side_offset
            angle = math.atan2(ball_v_from_kick_foot, ball_u)
            goal_u = ball_u - math.cos(angle) * distance
            goal_v = ball_v - math.sin(angle) * distance + side_offset
            ball_point = (goal_u, goal_v, angle, self._blackboard.world_model.base_footprint_frame)

        else:
            self._node.get_logger().error(f"Target {target} for go_to_ball action not implemented.")
            return

        # Create the goal pose message
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = ball_point[3]
        pose_msg.pose.position = Point(x=ball_point[0], y=ball_point[1], z=0.0)
        pose_msg.pose.orientation = quat_from_yaw(ball_point[2])

        # Convert the goal to the map frame
        pose_msg = self._blackboard.tf_buffer.transform(pose_msg, self._blackboard.map_frame)

        return pose_msg

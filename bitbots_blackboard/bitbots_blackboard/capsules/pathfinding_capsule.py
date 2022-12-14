from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from bitbots_blackboard.blackboard import BodyBlackboard

import math

import numpy as np
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.publisher import Publisher
from ros2_numpy import numpify
from std_msgs.msg import Empty
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from bitbots_utils.utils import get_parameters_from_other_node


class PathfindingCapsule:
    def __init__(self, blackboard: "BodyBlackboard", node: Node):
        self.node = node
        self._blackboard = blackboard
        self.map_frame: str = self.node.get_parameter('map_frame').value
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.position_threshold: str = self.node.get_parameter('body.pathfinding_position_threshold').value
        self.orientation_threshold: str = self.node.get_parameter('body.pathfinding_orientation_threshold').value
        self.direct_cmd_vel_pub: Optional[Publisher] = None
        self.pathfinding_pub: Optional[Publisher] = None
        self.pathfinding_cancel_pub: Optional[Publisher] = None
        self.ball_obstacle_active_pub: Optional[Publisher] = None
        self.approach_marker_pub: Optional[Publisher] = None
        self.goal: Optional[PoseStamped] = None
        self.avoid_ball: bool = True
        self.current_cmd_vel = Twist()
        self.orient_to_ball_distance: float = get_parameters_from_other_node(
            self.node,
            'bitbots_path_planning',
            'controller.orient_to_goal_distance')["controller.orient_to_goal_distance"]

    def publish(self, msg: PoseStamped):
        self.goal = msg
        self.pathfinding_pub.publish(msg)

    def get_goal(self) -> PoseStamped:
        return self.goal

    def cancel_goal(self):
        self.pathfinding_cancel_pub.publish(Empty())

    def cmd_vel_cb(self, msg: Twist):
        self.current_cmd_vel = msg

    def stop_walk(self):
        # send special command to walking to stop it
        msg = Twist()
        msg.angular.x = -1.0
        self.direct_cmd_vel_pub.publish(msg)

    def calculate_time_to_ball(self):
        # only send new request if previous request is finished or first update
        # also verify that the ball and the localization are reasonably recent/accurate
        if self._blackboard.world_model.ball_has_been_seen() and \
                self._blackboard.world_model.localization_precision_in_threshold():
            ball_target = self.get_ball_goal('map_goal', self._blackboard.config['ball_approach_dist'])
            own_position = self._blackboard.world_model.get_current_position_pose_stamped()
            self._blackboard.team_data.own_time_to_ball = self.time_to_ball_from_poses(own_position, ball_target)
        else:
            # since we can not get a reasonable estimate, we are lost and set the time_to_ball to a very high value
            self._blackboard.team_data.own_time_to_ball = 9999.0

    def time_to_ball_from_poses(self, own_pose: PoseStamped, goal_pose: PoseStamped) -> float:
        # calculate length of path
        start_point = own_pose.pose.position
        end_point = goal_pose.pose.position
        path_length = np.linalg.norm(numpify(start_point)[:2] - numpify(end_point)[:2])
        # if the robot is close to the ball it does not turn to walk to it
        if path_length < self.orient_to_ball_distance:
            _, _, start_theta = self._blackboard.world_model.get_current_position()
            goal_theta = euler_from_quaternion(numpify(goal_pose.pose.orientation))[2]
            start_goal_theta_diff = (abs(start_theta - goal_theta) + math.tau / 2) % math.tau - math.tau / 2
            start_goal_theta_cost = start_goal_theta_diff * self._blackboard.config[
                'time_to_ball_cost_start_to_goal_angle']
            total_cost = path_length * self._blackboard.config['time_to_ball_cost_per_meter'] + start_goal_theta_cost
        else:
            # calculate how much we need to turn to start walking along the path
            _, _, start_theta = self._blackboard.world_model.get_current_position()
            path_theta = math.atan2(end_point.y - start_point.y, end_point.x - start_point.x)
            start_theta_diff = (abs(start_theta - path_theta) + math.tau / 2) % math.tau - math.tau / 2
            # calculate how much we need to turn to turn at the end of the path
            goal_theta = euler_from_quaternion(numpify(goal_pose.pose.orientation))[2]
            goal_theta_diff = (abs(goal_theta - path_theta) + math.tau / 2) % math.tau - math.tau / 2
            start_theta_cost = start_theta_diff * self._blackboard.config['time_to_ball_cost_start_angle']
            goal_theta_cost = goal_theta_diff * self._blackboard.config['time_to_ball_cost_goal_angle']
            total_cost = path_length * self._blackboard.config['time_to_ball_cost_per_meter'] + \
                         start_theta_cost + goal_theta_cost
        return total_cost

    def get_ball_goal(self, target: str, distance: float) -> PoseStamped:

        if 'gradient_goal' == target:
            ball_x, ball_y = self._blackboard.world_model.get_ball_position_xy()

            goal_angle = self._blackboard.world_model.get_gradient_direction_at_field_position(ball_x, ball_y)

            goal_x = ball_x - math.cos(goal_angle) * distance
            goal_y = ball_y - math.sin(goal_angle) * distance

            ball_point = (goal_x, goal_y, goal_angle, self._blackboard.map_frame)

        elif 'map_goal' == target:
            goal_angle = self._blackboard.world_model.get_map_based_opp_goal_angle_from_ball()

            ball_x, ball_y = self._blackboard.world_model.get_ball_position_xy()

            if abs(ball_y) < self._blackboard.world_model.goal_width / 2:
                goal_angle = 0

            goal_x = ball_x - math.cos(goal_angle) * distance
            goal_y = ball_y - math.sin(goal_angle) * distance

            ball_point = (goal_x, goal_y, goal_angle, self._blackboard.map_frame)

        elif 'detection_goal' == target:

            x_dist = self._blackboard.world_model.get_detection_based_goal_position_uv()[0] - \
                     self._blackboard.world_model.get_ball_position_uv()[0]
            y_dist = self._blackboard.world_model.get_detection_based_goal_position_uv()[1] - \
                     self._blackboard.world_model.get_ball_position_uv()[1]

            goal_angle = math.atan2(y_dist, x_dist)

            ball_u, ball_v = self._blackboard.world_model.get_ball_position_uv()
            goal_u = ball_u + math.cos(goal_angle) * distance
            goal_v = ball_v + math.sin(goal_angle) * distance

            ball_point = (goal_u, goal_v, goal_angle, self._blackboard.world_model.base_footprint_frame)

        elif 'none' == target or 'current_orientation' == target:

            ball_u, ball_v = self._blackboard.world_model.get_ball_position_uv()
            ball_point = (ball_u, ball_v, 0, self._blackboard.world_model.base_footprint_frame)

        elif 'close' == target:

            ball_u, ball_v = self._blackboard.world_model.get_ball_position_uv()
            angle = math.atan2(ball_v, ball_u)
            ball_point = (ball_u, ball_v, angle, self._blackboard.world_model.base_footprint_frame)
        else:
            self.node.get_logger().error("Target %s for go_to_ball action not specified.", target)
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = ball_point[3]
        pose_msg.pose.position = Point(x=ball_point[0], y=ball_point[1], z=0.0)
        quaternion = quaternion_from_euler(0, 0, ball_point[2])
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        return pose_msg

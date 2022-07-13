import rclpy
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Time
import math
from rclpy.duration import Duration
import tf2_ros
import numpy as np
from ros2_numpy import numpify
from geometry_msgs.msg import PoseStamped, Point, Twist
from actionlib_msgs.msg import GoalID
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class PathfindingCapsule:
    def __init__(self, blackboard: "BodyBlackboard", node: Node):
        self.node = node
        self.map_frame = self.node.get_parameter('map_frame').get_parameter_value().string_value
        # Thresholds to determine whether the transmitted goal is a new one
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.position_threshold = self.node.get_parameter('body.pathfinding_position_threshold').get_parameter_value().double_value
        self.orientation_threshold = self.node.get_parameter('body.pathfinding_orientation_threshold').get_parameter_value().double_value
        self.direct_cmd_vel_pub = None  # type: Publisher
        self.pathfinding_pub = None  # type: Publisher
        self.pathfinding_cancel_pub = None  # type: Publisher
        self.path_to_ball_pub = None  # type: Publisher
        self.ball_obstacle_active_pub = None
        self.keep_out_area_pub = None
        self.approach_marker_pub = None
        self.goal = None  # type: PoseStamped
        self.current_pose = None  # type: PoseStamped
        self.status = -1  # Current status of movebase
        self.avoid_ball = True
        self.current_cmd_vel = Twist()
        self._blackboard = blackboard
        #self.orient_to_ball_distance = self.node.get_parameter("move_base.BBPlanner.orient_to_goal_distance").get_parameter_value().double_value
        self.orient_to_ball_distance = 0.5  # TODO: fill in new value once the planner is ported

    def publish(self, msg):
        # type: (PoseStamped) -> None
        self.status = -1
        map_goal = self.transform_goal_to_map(msg)
        if map_goal:
            self.goal = map_goal
            self.pathfinding_pub.publish(self.fix_rotation(map_goal))

    def transform_goal_to_map(self, msg):
        # type: (PoseStamped) -> PoseStamped
        # transform local goal to goal in map frame
        if msg.header.frame_id == self.map_frame:
            return msg
        else:
            try:
                msg.header.stamp = Time(seconds=0, nanoseconds=0, clock_type=ClockType.ROS_TIME).to_msg() #TODO: maybe weird
                map_goal = self.tf_buffer.transform(msg, self.map_frame, timeout=Duration(seconds=0.5))
                e = euler_from_quaternion((map_goal.pose.orientation.x, map_goal.pose.orientation.y,
                                           map_goal.pose.orientation.z, map_goal.pose.orientation.w))
                q = quaternion_from_euler(0, 0, e[2])
                map_goal.pose.orientation.x = q[0]
                map_goal.pose.orientation.y = q[1]
                map_goal.pose.orientation.z = q[2]
                map_goal.pose.orientation.w = q[3]
                map_goal.pose.position.z = 0.0
                return map_goal
            except Exception as e:
                self.node.get_logger().warn(e)
                return

    def fix_rotation(self, msg):
        # type: (PoseStamped) -> PoseStamped
        # this adds translatory movement to a rotation to fix a pathfinding issue
        if (msg.pose.position.x == 0 and msg.pose.position.y == 0 and
                not (msg.pose.orientation.x == 0 and msg.pose.orientation.y == 0 and msg.pose.orientation.z == 0)):
            msg.pose.position.x = 0.01
            msg.pose.position.y = 0.01
        return msg

    def feedback_callback(self, msg):
        # type: (PoseStamped) -> None
        self.current_pose = msg.feedback.base_position

    def status_callback(self, msg):
        self.status = msg.status.status

    def get_goal(self):
        # type: () -> PoseStamped
        return self.goal

    def get_current_pose(self):
        return self.current_pose

    def cancel_goal(self):
        self.pathfinding_cancel_pub.publish(GoalID())

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
            return None

    def time_to_ball_from_poses(self, own_pose: PoseStamped, goal_pose: PoseStamped):
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

    def get_ball_goal(self, target, distance):

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

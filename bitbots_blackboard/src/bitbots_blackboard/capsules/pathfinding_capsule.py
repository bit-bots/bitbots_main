import rospy
import math

import tf2_ros
import numpy as np
from ros_numpy import numpify
from geometry_msgs.msg import PoseStamped, Point, Twist
from actionlib_msgs.msg import GoalID
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.srv import GetPlanRequest


class PathfindingCapsule:
    def __init__(self, blackboard):
        self.map_frame = rospy.get_param('~map_frame', 'map')
        # Thresholds to determine whether the transmitted goal is a new one
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.position_threshold = rospy.get_param('behavior/body/pathfinding_position_threshold')
        self.orientation_threshold = rospy.get_param('behavior/body/pathfinding_orientation_threshold')
        self.direct_cmd_vel_pub = None  # type: rospy.Publisher
        self.pathfinding_pub = None  # type: rospy.Publisher
        self.pathfinding_cancel_pub = None  # type: rospy.Publisher
        self.path_to_ball_pub = None  # type: rospy.Publisher
        self.ball_obstacle_active_pub = None
        self.keep_out_area_pub = None
        self.approach_marker_pub = None
        self.goal = None  # type: PoseStamped
        self.current_pose = None  # type: PoseStamped
        self.status = -1  # Current status of movebase
        self.avoid_ball = True
        self.current_cmd_vel = Twist()
        self._blackboard = blackboard  # type: BodyBlackboard

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
                msg.header.stamp = rospy.Time(0)
                map_goal = self.tf_buffer.transform(msg, self.map_frame, timeout=rospy.Duration(0.5))
                e = euler_from_quaternion((map_goal.pose.orientation.x, map_goal.pose.orientation.y,
                                           map_goal.pose.orientation.z, map_goal.pose.orientation.w))
                q = quaternion_from_euler(0, 0, e[2])
                map_goal.pose.orientation.x = q[0]
                map_goal.pose.orientation.y = q[1]
                map_goal.pose.orientation.z = q[2]
                map_goal.pose.orientation.w = q[3]
                map_goal.pose.position.z = 0
                return map_goal
            except Exception as e:
                rospy.logwarn(e)
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
        if self._blackboard.world_model.ball_seen() and \
                self._blackboard.world_model.localization_precision_in_threshold():
            ball_target = self.get_ball_goal('map_goal', self._blackboard.config['ball_approach_dist'])
            own_position = self._blackboard.world_model.get_current_position_pose_stamped()
            self._blackboard.team_data.own_time_to_ball = self.time_to_ball_from_poses(own_position, ball_target)
        else:
            # since we can not get a reasonable estimate, we are lost and set the time_to_ball to a very high value
            if not self._blackboard.world_model.ball_seen():
                rospy.loginfo("time_to_ball: ball not seen for some time")
            if not self._blackboard.world_model.localization_precision_in_threshold():
                rospy.loginfo("time_to_ball: localization bad")
            self._blackboard.team_data.own_time_to_ball = 9999.0
            return None

    def time_to_ball_from_poses(self, own_pose: PoseStamped, goal_pose: PoseStamped):
        # calculate length of path
        start_point = own_pose.pose.position
        end_point = goal_pose.pose.position
        path_length = np.linalg.norm(numpify(start_point)[:2] - numpify(end_point)[:2])
        # if the robot is close to the ball it does not turn to walk to it
        if path_length < rospy.get_param("move_base/BBPlanner/orient_to_goal_distance", 1):
            _, _, start_theta = self._blackboard.world_model.get_current_position()
            goal_theta = euler_from_quaternion(numpify(goal_pose.pose.orientation))[2]
            start_goal_theta_diff = (abs(start_theta - goal_theta) + math.tau / 2) % math.tau - math.tau / 2
            start_goal_theta_cost = start_goal_theta_diff * self._blackboard.config[
                'time_to_ball_start_to_goal_angle_weight']
            total_cost = path_length + start_goal_theta_cost
            #rospy.logerr(f"Close to ball: start_goal_diff: {start_goal_theta_diff} " +
            #             f"weighted start_goal_diff: {start_goal_theta_cost}, " +
            #             f"path_length: {path_length}, " +
            #             f"total: {total_cost}")
        else:
            # calculate how much we need to turn to start walking along the path
            _, _, start_theta = self._blackboard.world_model.get_current_position()
            path_theta = math.atan2(end_point.y - start_point.y, end_point.x - start_point.x)
            start_theta_diff = (abs(start_theta - path_theta) + math.tau / 2) % math.tau - math.tau / 2
            # calculate how much we need to turn to turn at the end of the path
            goal_theta = euler_from_quaternion(numpify(goal_pose.pose.orientation))[2]
            goal_theta_diff = (abs(goal_theta - path_theta) + math.tau / 2) % math.tau - math.tau / 2
            start_theta_cost = start_theta_diff * self._blackboard.config['time_to_ball_start_angle_weight']
            goal_theta_cost = goal_theta_diff * self._blackboard.config['time_to_ball_goal_angle_weight']
            total_cost = path_length + start_theta_cost + goal_theta_cost
            #rospy.logerr(f"Far from ball: start_diff: {start_theta_diff}, goal_diff: {goal_theta_diff}, " +
            #             f"weighted start_diff: {start_theta_cost}, " +
            #             f"weighted goal_diff: {goal_theta_cost}, " +
            #             f"path_length: {path_length}, " +
            #             f"total: {total_cost}")
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
            rospy.logerr("Target %s for go_to_ball action not specified.", target)
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = ball_point[3]
        pose_msg.pose.position = Point(ball_point[0], ball_point[1], 0)
        quaternion = quaternion_from_euler(0, 0, ball_point[2])
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        return pose_msg

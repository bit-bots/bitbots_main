import rospy
import math

import tf2_ros
from geometry_msgs.msg import PoseStamped, Point
from actionlib_msgs.msg import GoalID
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.srv import GetPlan, GetPlanRequest
from nav_msgs.msg import Path
#from bitbots_blackboard.blackboard import BodyBlackboard


class PathfindingCapsule:
    def __init__(self, blackboard):
        self.map_frame = rospy.get_param('~map_frame', 'map')
        # Thresholds to determine whether the transmitted goal is a new one
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.position_threshold = rospy.get_param('behavior/body/pathfinding_position_threshold')
        self.orientation_threshold = rospy.get_param('behavior/body/pathfinding_orientation_threshold')
        self.pathfinding_pub = None  # type: rospy.Publisher
        self.pathfinding_cancel_pub = None  # type: rospy.Publisher
        self.ball_obstacle_active_pub = None
        self.approach_marker_pub = None
        self.goal = None  # type: PoseStamped
        self.current_pose = None # type: PoseStamped
        self.status = -1 # Current status of movebase
        self.avoid_ball = True
        self.__blackboard = blackboard  # type: BodyBlackboard
        self.get_plan_service = None
        self.path_to_ball = None # type: Path
        self.last_path_update = None  # type rospy.Time
        self.current_path_update = None  # type rospy.Time

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
        if msg.header.frame_id ==  self.map_frame:
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

    def get_new_path_to_ball(self):
        # only send new request if previous request is finished or first update
        if self.last_path_update is None or self.current_path_update is None or\
                self.current_path_update > self.last_path_update:
            ball_target = self.get_ball_goal('map_goal', self.__blackboard.config['ball_approach_dist'], 2)
            rospy.logerr(ball_target)
            own_position = self.__blackboard.world_model.get_current_position_pose_stamped()
            rospy.logerr(own_position)
            req = GetPlanRequest()
            req.goal = ball_target
            req.start = own_position
            self.get_plan_service(req)

    def path_to_ball_cb(self, response):
        self.last_path_update = self.current_path_update
        self.current_path_update = rospy.Time.now()
        self.path_to_ball = response.plan
        self.__blackboard.team_data.own_time_to_ball = self.calculate_time_to_ball()

    def calculate_time_to_ball(self):
        return rospy.Time.now().secs

    def get_ball_goal(self, target, distance, goal_width):

        if 'gradient_goal' == target:
            ball_x, ball_y = self.__blackboard.world_model.get_ball_position_xy()

            goal_angle = self.__blackboard.world_model.get_gradient_direction_at_field_position(ball_x, ball_y)

            goal_x = ball_x - math.cos(goal_angle) * distance
            goal_y = ball_y - math.sin(goal_angle) * distance

            ball_point = (goal_x, goal_y, goal_angle, self.__blackboard.map_frame)

        elif 'map_goal' == target:
            goal_angle = self.__blackboard.world_model.get_map_based_opp_goal_angle_from_ball()

            ball_x, ball_y = self.__blackboard.world_model.get_ball_position_xy()

            if abs(ball_y) < goal_width / 2:
                goal_angle = 0

            goal_x = ball_x - math.cos(goal_angle) * distance
            goal_y = ball_y - math.sin(goal_angle) * distance

            ball_point = (goal_x, goal_y, goal_angle, self.__blackboard.map_frame)

        elif 'detection_goal' == target:

            x_dist = self.__blackboard.world_model.get_detection_based_goal_position_uv()[0] - \
                     self.__blackboard.world_model.get_ball_position_uv()[0]
            y_dist = self.__blackboard.world_model.get_detection_based_goal_position_uv()[1] - \
                     self.__blackboard.world_model.get_ball_position_uv()[1]

            goal_angle = math.atan2(y_dist, x_dist)

            ball_u, ball_v = self.__blackboard.world_model.get_ball_position_uv()
            goal_u = ball_u + math.cos(goal_angle) * distance
            goal_v = ball_v + math.sin(goal_angle) * distance

            ball_point = (goal_u, goal_v, goal_angle, self.__blackboard.world_model.base_footprint_frame)

        elif 'none' == target or 'current_orientation' == target:

            ball_u, ball_v = self.__blackboard.world_model.get_ball_position_uv()
            ball_point = (ball_u, ball_v, 0, self.__blackboard.world_model.base_footprint_frame)

        elif 'close' == target:

            ball_u, ball_v = self.__blackboard.world_model.get_ball_position_uv()
            angle = math.atan2(ball_v, ball_u)
            ball_point = (ball_u, ball_v, angle, self.__blackboard.world_model.base_footprint_frame)
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

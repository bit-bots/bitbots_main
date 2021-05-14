import rospy
import tf2_ros as tf2
import math
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from actionlib_msgs.msg import GoalStatus

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class GoToBall(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GoToBall, self).__init__(blackboard, dsd, parameters)

        if 'target' not in parameters.keys():
            rospy.logerr('The parameter \'{}\' could not be used to decide whether map information is accesible'.format(
                parameters['target']))
        else:
            self.target = parameters['target']

        self.blocking = parameters.get('blocking', True)
        self.distance = parameters.get('distance', self.blackboard.config['ball_approach_dist'])


    def perform(self, reevaluate=False):

        if 'map_goal' == self.target:
            goal_angle = self.blackboard.world_model.get_map_based_opp_goal_angle_from_ball()

            ball_x, ball_y = self.blackboard.world_model.get_ball_position_xy()
            goal_x = ball_x - math.cos(goal_angle) * self.distance
            goal_y = ball_y - math.sin(goal_angle) * self.distance

            ball_point = (goal_x, goal_y, goal_angle, self.blackboard.map_frame)

        elif 'detection_goal' == self.target:

            x_dist = self.blackboard.world_model.get_detection_based_goal_position_uv()[0] - \
                     self.blackboard.world_model.get_ball_position_uv()[0]
            y_dist = self.blackboard.world_model.get_detection_based_goal_position_uv()[1] - \
                     self.blackboard.world_model.get_ball_position_uv()[1]

            goal_angle = math.atan2(y_dist, x_dist)

            ball_u, ball_v = self.blackboard.world_model.get_ball_position_uv()
            goal_u = ball_u + math.cos(goal_angle) * self.distance
            goal_v = ball_v + math.sin(goal_angle) * self.distance

            ball_point = (goal_u, goal_v, goal_angle, self.blackboard.world_model.base_footprint_frame)

        elif 'none' == self.target or 'current_orientation' == self.target:

            ball_u, ball_v = self.blackboard.world_model.get_ball_position_uv()
            ball_point = (ball_u, ball_v, 0, self.blackboard.world_model.base_footprint_frame)

        elif 'close' == self.target:

            ball_u, ball_v = self.blackboard.world_model.get_ball_position_uv()
            angle = math.atan2(ball_v, ball_u)
            ball_point = (ball_u, ball_v, angle, self.blackboard.world_model.base_footprint_frame)
        else:
            rospy.logerr("Target %s for go_to_ball action not specified.", self.target)
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

        self.blackboard.pathfinding.publish(pose_msg)

        approach_marker = Marker()
        approach_marker.pose.position.x = self.distance
        approach_marker.type = Marker.SPHERE
        approach_marker.action = Marker.MODIFY
        approach_marker.id = 1
        color = ColorRGBA()
        color.r = 1.0
        color.g = 1.0
        color.b = 1.0
        color.a = 1.0
        approach_marker.color = color
        approach_marker.lifetime = rospy.Duration(nsecs=0.5)
        scale = Vector3(0.2, 0.2, 0.2)
        approach_marker.scale = scale
        approach_marker.header.stamp = rospy.Time.now()
        approach_marker.header.frame_id = self.blackboard.world_model.base_footprint_frame

        self.blackboard.pathfinding.approach_marker_pub.publish(approach_marker)

        if self.blackboard.pathfinding.status in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED]  or not self.blocking:
            self.pop()

import rospy
import tf2_ros as tf2
import math
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler

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


    def perform(self, reevaluate=False):

        if 'map_goal' == self.target:
            goal_angle = self.blackboard.world_model.get_map_based_opp_goal_angle_from_ball()

            dist = 0.2 # TODO param

            goal_x = self.blackboard.world_model.ball_map.point.x - math.cos(goal_angle) * dist
            goal_y = self.blackboard.world_model.ball_map.point.y - math.sin(goal_angle) * dist

            ball_point = (goal_x, goal_y, goal_angle)

        elif 'far_map_goal' == self.target:
            goal_angle = self.blackboard.world_model.get_map_based_opp_goal_angle_from_ball()

            dist = 0.5 # TODO param

            goal_x = self.blackboard.world_model.ball_map.point.x - math.cos(goal_angle) * dist
            goal_y = self.blackboard.world_model.ball_map.point.y - math.sin(goal_angle) * dist

            ball_point = (goal_x, goal_y, goal_angle)

        elif 'detection_goal' == self.target:
            x_dist = self.blackboard.world_model.get_detection_based_goal_position_uv()[0] - \
                     self.blackboard.world_model.get_ball_position_uv()[0]
            y_dist = self.blackboard.world_model.get_detection_based_goal_position_uv()[1] - \
                     self.blackboard.world_model.get_ball_position_uv()[1]

            goal_angle = math.atan2(y_dist, x_dist)

            dist = 0.2 # TODO param

            goal_x = self.blackboard.world_model.ball_map.point.x + math.cos(goal_angle) * dist
            goal_y = self.blackboard.world_model.ball_map.point.y + math.sin(goal_angle) * dist

            ball_point = (goal_x, goal_y, goal_angle)

        elif 'none' == self.target or 'current_orientation' == self.target:
            ball_u, ball_v = self.blackboard.world_model.get_ball_position_uv()
            ball_point = (ball_u, ball_v, 0)
        elif 'close' == self.target:
            ball_u, ball_v = self.blackboard.world_model.get_ball_position_uv()
            angle = math.atan2(ball_v, ball_u)
            ball_point = (ball_u, ball_v, angle)
        else:
            rospy.logerr("Target %s for go_to_ball action not specified.", self.target)
            return
        point = Point(*ball_point)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()

        pose_msg.header.frame_id = self.blackboard.map_frame

        pose_msg.pose.position = Point(point.x, point.y, 0)

        quaternion = quaternion_from_euler(0, 0, point.z)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        self.blackboard.pathfinding.publish(pose_msg)

        if self.blackboard.pathfinding.status == self.blackboard.pathfinding.SUCCEEDED or not self.blocking:
            self.pop()

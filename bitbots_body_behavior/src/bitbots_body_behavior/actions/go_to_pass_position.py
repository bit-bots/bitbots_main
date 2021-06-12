import math

import rospy
from actionlib_msgs.msg import GoalStatus
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Point

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from tf.transformations import quaternion_from_euler


class AbstractGoToPassPosition(AbstractActionElement):
    def __init__(self, blackboard, dsd, accept, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.pass_pos_x = rospy.get_param("pass_position_x", 1)
        self.pass_pos_y = rospy.get_param("pass_position_y", 1)
        self.accept = accept

    def perform(self, reevaluate=False):
        # get ball pos
        ball_pos = self.blackboard.world_model.get_ball_position_xy()
        our_pose = self.blackboard.world_model.get_current_position()

        # decide on side
        if our_pose[1] - ball_pos[1] < 0:
            if -math.tau / 4 < our_pose[2] < math.tau / 4:
                # the ball is left of us
                side_sign = -1
            else:
                # ball is right of us
                side_sign = 1
        else:
            if -math.tau / 4 < our_pose[2] < math.tau / 4:
                side_sign = 1
            else:
                side_sign = -1

        # compute goal
        goal_x = ball_pos[0]
        if self.accept:
            goal_x += self.pass_pos_x

        goal_y = ball_pos[1] + side_sign * self.pass_pos_y
        goal_yaw = self.blackboard.world_model.get_gradient_direction_at_field_position(goal_x, goal_y)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.blackboard.map_frame
        pose_msg.pose.position.x = goal_x
        pose_msg.pose.position.y = goal_y
        quaternion = quaternion_from_euler(0, 0, goal_yaw)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        self.blackboard.pathfinding.publish(pose_msg)

        if self.blackboard.pathfinding.status in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED]:
            self.pop()


class GoToPassPreparePosition(AbstractGoToPassPosition):
    """
    Go to a position 1m left or right from the ball (whichever is closer) as preparation for a pass
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, False, parameters)


class GoToPassAcceptPosition(AbstractGoToPassPosition):
    """
        Go to a position forward of the ball to accept a pass from another robot.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, True, parameters)

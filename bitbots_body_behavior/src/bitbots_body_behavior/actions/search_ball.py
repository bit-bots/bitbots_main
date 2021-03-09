from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import rospy
import math


class SearchBall(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(SearchBall, self).__init__(blackboard, dsd, parameters)
        self.time_last_turn = rospy.Time.now()

    def perform(self, reevaluate=False):
        # TODO make parameter value
        if self.time_last_turn < self.blackboard.world_model.ball_last_seen():
            self.time_last_turn = rospy.Time.now()
        if rospy.Time.now() - self.time_last_turn > rospy.Duration(20):
            # remember that we turned around
            self.time_last_turn = rospy.Time.now()

            # goal to turn by 90 deg left
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = 'base_footprint'

            quaternion = quaternion_from_euler(0, 0, - math.pi / 2.0)

            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            self.blackboard.pathfinding.publish(pose_msg)


class SearchBallPenalty(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(SearchBallPenalty, self).__init__(blackboard, dsd, parameters)
        self.time_last_movement = rospy.Time.now()

    def perform(self, reevaluate=False):
        self.blackboard.blackboard.set_head_duty(HeadMode.BALL_MODE_PENALTY)
        # TODO make parameter value
        if rospy.Time.now() - self.time_last_movement > rospy.Duration(3):
            self.time_last_movement = rospy.Time.now()

            # goal to go straight
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = 'base_footprint'

            pose_msg.pose.position.x = 0.75

            quaternion = quaternion_from_euler(0, 0, 0)

            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            self.blackboard.pathfinding.publish(pose_msg)

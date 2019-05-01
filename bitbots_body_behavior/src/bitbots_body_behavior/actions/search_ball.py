from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode
import rospy
import tf


class SearchBall(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(SearchBall, self).__init__(blackboard, dsd, parameters)
        self.time_last_turn = rospy.Time.now()

    def perform(self, reevaluate=False):
        self.blackboard.blackboard.set_head_duty(HeadMode.BALL_MODE)
        # TODO make parameter value
        if rospy.Time.now() - self.time_last_turn > 10:
            # remember that we turned around
            self.time_last_turn = rospy.Time.now()

            # goal to turn by 90 deg left
            pose_msg = PoseStamped() #todo import
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = 'base_footprint'

            quaternion = tf.transformations.quaternion_from_euler(0, 0, 1.57)

            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]

            self.blackboard.pathfinding.publish(pose_msg)
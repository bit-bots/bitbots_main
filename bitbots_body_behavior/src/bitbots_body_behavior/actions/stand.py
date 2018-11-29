import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point

from bitbots_dsd.abstract_action_element import AbstractActionElement


class Stand(AbstractActionElement):
    def perform(self, reevaluate=False):
        # TODO evaluate whether we use only move base
        if not self.blackboard.config['use_move_base']:
            self.blackboard.pathfinding.pub_simple_pathfinding(0, 0)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'base_footprint'

        pose_msg.pose.position = Point(0, 0, 0)
        pose_msg.pose.orientation = Quaternion(0, 0, 0, 1)

        self.blackboard.pathfinding.call_action(pose_msg)

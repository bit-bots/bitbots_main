import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class Stand(AbstractActionElement):
    def perform(self, reevaluate=False):
        # TODO evaluate whether we use only move base
        if not self.blackboard.config['use_move_base']:
            self.blackboard.pathfinding.pub_simple_pathfinding(0, 0)

        else:
            self.blackboard.pathfinding.cancel_path()

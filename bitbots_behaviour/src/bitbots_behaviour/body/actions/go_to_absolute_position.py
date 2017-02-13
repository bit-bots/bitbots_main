"""
GoToAbsolutePosition
^^^^^^^^^^^^^^^^^^^^

This Module is responsible for walking to a specific point with a specific
orientation within the absolute coordinate system.

"""

from bitbots_stackmachine.abstract_action_module import AbstractActionModule
from geometry_msgs.msg import Pose2D

from model.connector import Connector


class GoToAbsolutePosition(AbstractActionModule):
    def __init__(self, connector: Connector, args=None):
        AbstractActionModule.__init__(self, connector, args)
        self.target_x = args[0]
        self.target_y = args[1]
        self.target_o = args[2]

        self.tolerance = 400  # TODO parameterisierbar

        self.align_on_zero = False

    def perform(self, connector: Connector, reevaluate=False):

        if connector.world_model.get_distance_to_xy(self.target_x, self.target_y) > self.tolerance:
            p = Pose2D()
            p.x = self.target_x
            p.y = self.target_y
            p.theta = self.target_o

            connector.walking.pub_walking_objective.publish(p)
        else:
            self.pop()


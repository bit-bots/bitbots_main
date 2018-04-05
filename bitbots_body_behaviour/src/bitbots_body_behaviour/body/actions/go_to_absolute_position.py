"""
GoToAbsolutePosition
^^^^^^^^^^^^^^^^^^^^

This Module is responsible for walking to a specific point with a specific
orientation within the absolute coordinate system.

"""

import math

from bitbots_stackmachine.abstract_action_module import AbstractActionModule
from geometry_msgs.msg import Twist

from bitbots_common.connector.connector import BodyConnector


class GoToAbsolutePosition(AbstractActionModule):
    def __init__(self, connector: BodyConnector, args=None):
        AbstractActionModule.__init__(self, connector, args)
        self.target_x = args[0]
        self.target_y = args[1]
        self.target_o = args[2]

        self.tolerance = 0  # TODO parameterisierbar

        self.align_on_zero = False

    def perform(self, connector: BodyConnector, reevaluate=False):

        print("Walking to %s", [self.target_x, self.target_y, self.target_o])

        if connector.world_model.get_distance_to_xy(self.target_x, self.target_y) > self.tolerance:
            p = Twist()
            p.linear.x = math.copysign(0.04, self.target_x) if self.target_x is not 0 else 0
            p.linear.y = math.copysign(0.02, self.target_y) if self.target_y is not 0 else 0
            p.angular.z = self.target_o

            print("Publishing!")
            connector.walking.pub_walkin_params.publish(p)
        else:
            self.pop()


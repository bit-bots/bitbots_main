"""
TurnToAbsoluteDirection
^^^^^^^^^^^^^^

Turns the robot to a absolute direction. Its called with direction in degree (0 is opposite ground line) and threshold.
The threshold determines how precise the robot tries to get to this angle. 10 degree threshold will result in up to
10 degree error in each direction, making a range of 20 degree for the robot position.

History:
* 06.12.14: Created (Marc Bestmann)
"""
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule

from bitbots_common.connector.connector import BodyConnector


class TurnToAbsoluteDirection(AbstractDecisionModule):
    def __init__(self, connector: BodyConnector, args):
        super(TurnToAbsoluteDirection, self).__init__(connector)
        self.goal_direction = args[0]
        self.threshold = args[1]

    def perform(self, connector: BodyConnector, reevaluate=False):

        current_direction = connector.world_model.get_current_position()[2]

        if abs(self.goal_direction - current_direction) < self.threshold:
            # we reached the direction exact enough
            connector.walking.stop_walking()
            return self.pop()
        else:
            if current_direction < self.goal_direction:
                connector.walking.start_walking_plain(0, 5, 0)
            else:
                connector.walking.start_walking_plain(0, -5, 0)

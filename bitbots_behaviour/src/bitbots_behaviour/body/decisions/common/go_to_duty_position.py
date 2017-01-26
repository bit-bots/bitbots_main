"""
GoToDutyPosition
^^^^^^^^^^^^^^^^

Go back to the position where a robot with this duty should stand.

History:
* 05.12.14: Created (Marc Bestmann & Nils Rokita)
"""
from stackmachine.abstract_decision_module import AbstractDecisionModule

from body.actions.go_to_absolute_position import GoToAbsolutePosition
from stackmachine.model import Connector


class GoToDutyPosition(AbstractDecisionModule):
    def __init__(self, _):
        super(GoToDutyPosition, self).__init__()
        self.length = config["field"]["length"]
        self.width = config["field"]["width"]
        self.goalie_position = config["Behaviour"]["Common"]["Positions"]["goalie"]
        self.teamplayer_position = config["Behaviour"]["Common"]["Positions"]["teamPlayer"]
        self.defender_position = config["Behaviour"]["Common"]["Positions"]["defender"]
        self.center_position = config["Behaviour"]["Common"]["Positions"]["center"]
        self.threshold = config["Behaviour"]["Common"]["positioningThreshold"]

    def perform(self, connector: Connector, reevaluate=False):
        duty = connector.blackboard.get_duty()

        if duty == "Goalie":
            position = (self.goalie_position[0] * self.length, self.goalie_position[1] * self.width)
        elif duty == "TeamPlayer":
            position = (self.teamplayer_position[0] * self.length, self.teamplayer_position[1] * self.width)
        elif duty == "Defender":
            position = (self.defender_position[0] * self.length, self.defender_position[1] * self.width)
        elif duty == "Center":
            position = (self.center_position[0] * self.length, self.center_position[1] * self.width)
        else:
            position = (-0.25 * self.length, 0)  # this is the middle point of our own half

        if connector.world_model.get_distance_to_xy(position[0], position[1]) > self.threshold:
            say("Go to duty position")
            return self.push(GoToAbsolutePosition, (position, True))
        else:
            say("I am at position")
            return self.pop()

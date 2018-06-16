# -*- coding:utf-8 -*-
"""
GoToDutyPosition
^^^^^^^^^^^^^^^^

Go back to the position where a robot with this duty should stand.

History:
* 05.12.14: Created (Marc Bestmann & Nils Rokita)
"""
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from humanoid_league_msgs.msg import TeamData
from bitbots_body_behaviour.body.actions.go_to import GoToAbsolutePosition


class GoToDutyPosition(AbstractDecisionModule):
    def __init__(self, connector, _):
        super(GoToDutyPosition, self).__init__(connector)
        self.half_length = connector.config["Body"]["Common"]["Field"]["length"] / 2.0
        self.half_width = connector.config["Body"]["Common"]["Field"]["width"] / 2.0
        self.goalie_position = connector.config["Body"]["Common"]["Positions"]["goalie"]
        self.teamplayer_position = connector.config["Body"]["Common"]["Positions"]["teamPlayer"]
        self.defender_position = connector.config["Body"]["Common"]["Positions"]["defender"]
        self.center_position = connector.config["Body"]["Common"]["Positions"]["center"]
        self.threshold = connector.config["Body"]["Common"]["positioningThreshold"]

    def perform(self, connector, reevaluate=False):
        role = connector.team_data.get_role()[0]

        if role == TeamData.ROLE_GOALIE:
            position = (self.goalie_position[0] * self.half_length, self.goalie_position[1] * self.half_width, 0)
        elif role == TeamData.ROLE_STRIKER:
            position = (self.teamplayer_position[0] * self.half_length, self.teamplayer_position[1] * self.half_width, 0)
        elif role == TeamData.ROLE_DEFENDER:
            position = (self.defender_position[0] * self.half_length, self.defender_position[1] * self.half_width, 0)
        elif role == TeamData.ROLE_SUPPORTER:
            position = (self.center_position[0] * self.half_length, self.center_position[1] * self.half_width, 0)
        else:
            position = (-0.5 * self.half_length, 0, 0)  # this is the middle point of our own half

        if connector.world_model.get_distance_to_xy(position[0], position[1]) > self.threshold:
            connector.speaker.say("Go to duty position")
            return self.push(GoToAbsolutePosition, position)
        else:
            connector.speaker.say("I am at position")
            return self.pop()

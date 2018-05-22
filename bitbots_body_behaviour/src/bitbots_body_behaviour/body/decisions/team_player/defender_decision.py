# -*- coding:utf-8 -*-
"""
DefenderDecision
^^^^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

History:
* 24.3.14: Created (Martin Poppinga)

Decides what a defender does
"""
import rospy

from bitbots_body_behaviour.body.actions.wait import Wait
from bitbots_body_behaviour.body.decisions.one_time_kicker.one_time_kicker_decision import OneTimeKickerDecision
from bitbots_body_behaviour.body.decisions.team_player.defender_position_decider import DefenderPositionDecider
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class DefenderDecision(AbstractDecisionModule):
    def __init__(self, connector, _):
        super(DefenderDecision, self).__init__(connector, _)

        self.toggle_one_time_defender = connector.config["Body"]["Behaviour"]["Toggles"]["Fieldie"]["oneTimeDefender"]
        self.go_striker_range = connector.config["Body"]["Behaviour"]["Fieldie"]["Defender"]["goStrikerRange"]
        self.go_striker_time = connector.config["Body"]["Behaviour"]["Fieldie"]["Defender"]["goStrikerTime"]
        self.ball_history_length = connector.config["Body"]["Behaviour"]["Fieldie"]["Defender"]["ballHistoryLenght"]
        self.first_first_time = None
        self.required_number = connector.config["Body"]["Behaviour"]["Fieldie"]["Defender"]["requiredNumberTrues"]
        self.start_time = rospy.get_time()
        self.wait_at_start = connector.config["Body"]["Behaviour"]["Fieldie"]["Defender"]["waitAtStart"]
        self.ball_distance_history = []
        self.started = None
        self.start = rospy.get_time()

    def vote_for_switch_to_striker(self, connector):
        if (connector.personal_model.ball_seen() and
                connector.personal_model.get_ball_relative()[0] < self.go_striker_range):
            self.ball_distance_history.append(True)
        else:
            self.ball_distance_history.append(False)
        if len(self.ball_distance_history) > self.ball_history_length:
            self.ball_distance_history = self.ball_distance_history[1:]
        number_trues = len([e for e in self.ball_distance_history if e])
        return number_trues

    def perform(self, connector, reevaluate=False):
        number_of_times_voted_for_yes = 0
        if self.is_waiting_period_over():
            number_of_times_voted_for_yes = self.vote_for_switch_to_striker(connector)

        if number_of_times_voted_for_yes > self.required_number:
            return self.push(OneTimeKickerDecision)
        else:
            # Standard defender is moving in front of own goal
            if self.toggle_one_time_defender:
                return self.push(Wait, 1)
            else:
                return self.push(DefenderPositionDecider)

    def get_reevaluate(self):
        return True

    def is_waiting_period_over(self):
        return rospy.get_time() > self.start + self.wait_at_start

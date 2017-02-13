# -*- coding:utf-8 -*-
"""
DefenderDecision
^^^^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

History:
* 24.3.14: Created (Martin Poppinga)

Decides what a defender does
"""
import time

from body.actions.wait import Wait
from body.decisions.common.corridor import DefenderCorridor
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class DefenderDecision(AbstractDecisionModule):
    def __init__(self, _):
        super(DefenderDecision, self).__init__()

        self.toggle_one_time_defender = config["Toggles"]["Fieldie"]["oneTimeDefender"]
        self.go_striker_range = config["Fieldie"]["Defender"]["goStrikerRange"]
        self.go_striker_time = config["Fieldie"]["Defender"]["goStrikerTime"]
        self.ball_history_length = config["Fieldie"]["Defender"]["ballHistoryLenght"]
        self.first_first_time = None
        self.required_number = config["Fieldie"]["Defender"]["requiredNumberTrues"]
        self.start_time = time.time()
        self.wait_at_start = config["Fieldie"]["Defender"]["waitAtStart"]
        self.ball_distance_history = []
        self.started = None
        self.start = time.time()

    def vote_for_switch_to_striker(self, connector):
        if (connector.raw_vision_capsule().ball_seen() and connector.raw_vision_capsule().get_ball_info("u") <
            self.go_striker_range):
            self.ball_distance_history.append(True)
        else:
            self.ball_distance_history.append(False)
        if len(self.ball_distance_history) > self.ball_history_length:
            self.ball_distance_history = self.ball_distance_history[1:]
        number_trues = len([e for e in self.ball_distance_history if e])
        return number_trues

    def perform(self, connector, reevaluate=False):
        number_of_times_voted_for_yes = 0
        if connector.raw_vision_capsule().is_new_frame() and self.is_waiting_period_over():
            number_of_times_voted_for_yes = self.vote_for_switch_to_striker(connector)

        if number_of_times_voted_for_yes > self.required_number:
            become_one_time_kicker(connector)
            return self.interrupt()
        else:
            # Standard defender is moving in front of own goal
            if self.toggle_one_time_defender:
                return self.push(Wait, 1)
            else:
                return self.push(DefenderCorridor)

    def get_reevaluate(self):
        return True

    def is_waiting_period_over(self):
        return time.time() > self.start + self.wait_at_start

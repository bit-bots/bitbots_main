# -*- coding:utf-8 -*-
"""
Corridor
^^^^^^^^^^^^^^^^

.. moduleauthor:: Benjamin Kuffel <2kuffel@informatik.uni-hamburg.de>
                    Lars Thoms <2thoms@informatik.uni-hamburg.de>
                    Alexander Happel <2happel@informatik.uni-hamburg.de>
                    Judith Hartfill <2hartfill@informatik.uni-hamburg.de>

History:
* 19.8.14: Created (Benjamin Kuffel, Lars Thoms)
* 22.8.14: Test abgeschlossen und refactored

The robot stays in its corridor and finds back if it had leaved it.
"""

import math
import time

from bitbots_common.connector.capsules.walking_capsule import WalkingCapsule
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from body.actions.plain_walk_action import PlainWalkAction
from body.actions.wait import Wait
from body.decisions.team_player.defender_position_decider import DefenderPositionDecider
import rospy

class AbstractCorridor(AbstractDecisionModule):
    def __init__(self, connector, _):
        super(AbstractCorridor, self).__init__(connector)

        self.corridor_u_start = int(config["length"]) * (1.0 / 9.0)
        self.corridor_u_end = int(config["length"]) * (2.0 / 9.0)
        self.corridor_v = int(config["width"]) * (1.0 / 3.0)
        self.corridor_v_diff = 100
        self.field_length = int(config["length"])
        self.ang_threshold = 10
        self.goal_pos_v = 0
        self.goal_pos_u = 0
        self.own_goal = None
        self.opp_goal = None
        self.timestamp_goal = 0
        self.wait_goal_time = 4

    def perform(self, connector, reevaluate=False):
        self.own_goal = connector.filtered_vision_capsule().get_local_goal_model_own_goal()
        self.opp_goal = connector.filtered_vision_capsule().get_local_goal_model_opp_goal()
        if self.timestamp_goal < (int(rospy.get_time()) - self.wait_goal_time):
            self.timestamp_goal = int(rospy.get_time())

        # "own_pos" contains the information, at which hight the robot is in relation to his own goal
        own_hypotenuse = math.sqrt(self.own_goal[0] ** 2 + self.own_goal[1] ** 2)
        opp_hypotenuse = math.sqrt(self.opp_goal[0] ** 2 + self.opp_goal[1] ** 2)
        self.goal_pos_v = (math.sqrt(
            2 * (own_hypotenuse ** 2 * opp_hypotenuse ** 2 +
                 opp_hypotenuse ** 2 * self.field_length ** 2 +
                 self.field_length ** 2 * own_hypotenuse ** 2)
            - (own_hypotenuse ** 4 + opp_hypotenuse ** 4 + self.field_length ** 4))
                           / (2 * self.field_length))
        self.goal_pos_u = math.sqrt(own_hypotenuse ** 2 - self.goal_pos_v ** 2)

        # if he is not in his corridor
        if self._is_outside():

            # Align the robot to the lines
            if math.fabs(math.fabs(self.own_goal[1]) - math.fabs(self.opp_goal[1])) > self.ang_threshold:

                # turn clockwise
                if self.own_goal[1] > self.opp_goal[1]:
                    self.debug("UHRZEIGER")
                    return self.push(PlainWalkAction, [
                        [WalkingCapsule.ZERO, WalkingCapsule.MEDIUM_ANGULAR_LEFT, WalkingCapsule.ZERO, 4]])

                # turn counterclockwise
                else:
                    self.debug("GEGENUHRZEIGER")
                    return self.push(PlainWalkAction, [
                        [WalkingCapsule.ZERO, WalkingCapsule.MEDIUM_ANGULAR_RIGHT, WalkingCapsule.ZERO, 4]])

            # is the robot too far aherad ?
            elif self._is_too_far():
                self.debug("VORNE")
                return self.push(PlainWalkAction,
                                 [[WalkingCapsule.SLOW_BACKWARD, WalkingCapsule.ZERO, WalkingCapsule.ZERO, 4]])

            # is the robot too far behind?
            elif self._is_too_near():
                self.debug("HINTEN")
                return self.push(PlainWalkAction,
                                 [[WalkingCapsule.SLOW_FORWARD, WalkingCapsule.ZERO, WalkingCapsule.ZERO, 4]])

            # is the robot too lateral
            elif self._is_too_lateral():
                self.debug("SEITLICH")
                self.debug(str(self.corridor_v))

                # too left (positiv v-value)
                if self.own_goal[1] < 0:
                    return self.push(PlainWalkAction, [
                        [WalkingCapsule.ZERO, WalkingCapsule.ZERO, WalkingCapsule.SLOW_SIDEWARDS_RIGHT, 4]])

                # too right (negativ v-value)
                else:
                    return self.push(PlainWalkAction, [
                        [WalkingCapsule.ZERO, WalkingCapsule.ZERO, WalkingCapsule.SLOW_SIDEWARDS_LEFT, 4]])
        else:
            return self.act(connector)

    def act(self, connector):
        pass

    def _is_too_far(self):
        return self.goal_pos_u > self.corridor_u_end

    def _is_too_near(self):
        return self.goal_pos_u < self.corridor_u_start

    def _is_too_lateral(self):
        return self.goal_pos_v > self.corridor_v

    def _is_outside(self):
        return self._is_too_far() or self._is_too_near() or self._is_too_lateral()

    def get_reevaluate(self):
        return True


class DefenderCorridor(AbstractCorridor):
    def act(self, connector):
        return self.push(DefenderPositionDecider)


class CenterCorridor(AbstractCorridor):
    def __init__(self, args):
        super(CenterCorridor, self).__init__(args)
        config = get_config()["field"]
        self.corridor_u_start = int(config["length"]) * (3.5 / 9.0)
        self.corridor_u_end = int(config["length"]) * (5.5 / 9.0)
        self.ball_distance_history = []
        config = get_config()["Behaviour"]
        self.start = rospy.get_time()
        self.go_striker_range = config["Fieldie"]["Defender"]["goStrikerRange"]
        self.go_striker_time = config["Fieldie"]["Defender"]["goStrikerTime"]
        self.ball_history_length = config["Fieldie"]["Defender"]["ballHistoryLenght"]
        self.wait_at_start = config["Fieldie"]["Defender"]["waitAtStart"]
        self.required_number = config["Fieldie"]["Defender"]["requiredNumberTrues"]

    def act(self, connector):
        number_of_times_voted_for_yes = 0
        if connector.raw_vision_capsule().is_new_frame() and self.is_waiting_period_over():
            number_of_times_voted_for_yes = self.vote_for_switch_to_striker(connector)

        if number_of_times_voted_for_yes > self.required_number:
            connector.speaker.say("Going to Striker! Attack!")
            connector.set_duty("Striker")
            return self.interrupt()
        else:
            return self.push(Wait)

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

    def is_waiting_period_over(self):
        return rospy.get_time() > self.start + self.wait_at_start

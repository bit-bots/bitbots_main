# -*- coding:utf-8 -*-
"""
StandsCorrectDecision
^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

History:
* 3.2.15: Created (Martin Poppinga)
"""
import math
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.actions.align_on_ball import AlignOnBall
from bitbots.modules.behaviour.body.actions.align_to_goal import AlignToGoal
from bitbots.modules.behaviour.body.decisions.common.hack_align import HackAlign
from bitbots.modules.behaviour.body.decisions.common.kick_decision import KickDecisionCommon
from bitbots.util import get_config

config = get_config()


class StandsCorrectDecision(AbstractDecisionModule):
    """
    Decides if the robot stands correct and takes care if it doesn't
    """

    def __init__(self, _):
        super(StandsCorrectDecision, self).__init__()
        self.toggle_align_to_goal = config["Behaviour"]["Toggles"]["Fieldie"]["alignToGoal"]
        self.toggle_use_side_kick_in_game = config["Behaviour"]["Toggles"]["Fieldie"]["useSideKickInGame"]
        self.toggle_hack_align = config["Behaviour"]["Toggles"]["Fieldie"]["hackAlign"]
        self.config_kickalign_v = config["Behaviour"]["Fieldie"]["kickAlign"]

    def perform(self, connector, reevaluate=False):

        # get data
        opp_goal_u = connector.filtered_vision_capsule().get_local_goal_model_opp_goal()[0]
        opp_goal_v = connector.filtered_vision_capsule().get_local_goal_model_opp_goal()[1]
        opp_goal_deg = math.degrees(math.atan2(opp_goal_u, opp_goal_v))

        # todo check if left-right is correct
        opp_goal_left_post_u = connector.filtered_vision_capsule().get_local_goal_model_opp_goal_posts()[1][0]
        opp_goal_left_post_v = connector.filtered_vision_capsule().get_local_goal_model_opp_goal_posts()[1][1]
        opp_goal_left_post_deg = math.degrees(math.atan2(opp_goal_left_post_u, opp_goal_left_post_v))

        opp_goal_right_post_u = connector.filtered_vision_capsule().get_local_goal_model_opp_goal_posts()[0][0]
        opp_goal_right_post_v = connector.filtered_vision_capsule().get_local_goal_model_opp_goal_posts()[0][1]
        opp_goal_right_post_deg = math.degrees(math.atan2(opp_goal_right_post_u, opp_goal_right_post_v))

        # Align sidewards to the ball
        # if abs(connector.filtered_vision_capsule().get_local_goal_model_ball()[1]) > self.config_kickalign_v:
        if abs(connector.raw_vision_capsule().get_ball_info(
                "v")) > self.config_kickalign_v:  # todo wieder gefilterte daten verwenden

            return self.push(AlignOnBall)

        # Align to the goal
        elif self.toggle_hack_align and connector.get_duty() not in ["PenaltyKickFieldie", "ThrowIn"]:
            return self.push(HackAlign)

        elif self.toggle_align_to_goal and not connector.blackboard_capsule().has_stopped_aligning():

            # Not using a sidekick. Always tries to align for normal kick
            if not self.toggle_use_side_kick_in_game:

                # Goal is in front and I align the way I can score
                if opp_goal_right_post_v < 0 < opp_goal_left_post_v:
                    # the 0 (axis) is between the two posts
                    # Already aligned
                    return self.action_stands_correct(connector)

            else:
                # Also possible to use a sidekick
                if opp_goal_right_post_v < 0 < opp_goal_left_post_v:
                    # allready aligned for frontkick
                    return self.action_stands_correct(connector)

                elif opp_goal_v > 0:
                    # goal is on the left
                    if opp_goal_right_post_u < 0 < opp_goal_left_post_u:
                        # aligned for sidekick
                        return self.action_stands_correct(connector)

                elif opp_goal_v < 0:
                    # goal is on the right
                    if opp_goal_right_post_u > 0 > opp_goal_left_post_u:
                        # aligned for sidekick
                        return self.action_stands_correct(connector)

            return self.push(AlignToGoal)

        # Stands correct
        else:
            return self.action_stands_correct(connector)

    def action_stands_correct(self, connector):
        connector.blackboard_capsule().stop_aligning()
        return self.push(KickDecisionCommon)

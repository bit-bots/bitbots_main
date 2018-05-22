# -*- coding:utf-8 -*-
"""
StandsCorrectDecision
^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <robocup@poppinga.xyz>

"""

from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule

import rospy
from bitbots_body_behaviour.body.actions.align_on_ball import AlignOnBall
from bitbots_body_behaviour.body.actions.align_to_goal import AlignToGoal
from bitbots_body_behaviour.body.decisions.common.kick_decision import KickDecisionCommon


class StandsCorrectDecision(AbstractDecisionModule):
    """
    Decides if the robot stands correct and takes care if it doesn't
    """

    def __init__(self, connector, _):
        super(StandsCorrectDecision, self).__init__(connector)
        self.toggle_align_to_goal = connector.config["Body"]["Toggles"]["Fieldie"]["alignToGoal"]
        self.toggle_use_side_kick_in_game = connector.config["Body"]["Toggles"]["Fieldie"]["useSideKickInGame"]
        self.toggle_hack_align = connector.config["Body"]["Toggles"]["Fieldie"]["hackAlign"]
        self.config_kickalign_v = connector.config["Body"]["Fieldie"]["kickAlign"]

    def perform(self, connector, reevaluate=False):

        # get data
        left_post = connector.world_model.get_opp_goal_left_post_uv()
        right_post = connector.world_model.get_opp_goal_right_post_uv()

        # Align sidewards to the ball
        if abs(connector.world_model.get_ball_position_uv()[1]) > self.config_kickalign_v:
            # todo wieder gefilterte daten verwenden
            goal = connector.world_model.get_opp_goal_center_xy()
            direction = atan2(goal[1], goal[0])
            return self.push(GoToBall, direction)

        # Align to the goal
        elif self.toggle_align_to_goal and not connector.blackboard_capsule().has_stopped_aligning():

            # Not using a sidekick. Always tries to align for normal kick
            if not self.toggle_use_side_kick_in_game:

                # Goal is in front and I align the way I can score
                if left_post[1] > 0 > right_post[1]:
                    # the 0 (axis) is between the two posts
                    # Already aligned
                    return self.action_stands_correct(connector)

            else:
                # Also possible to use a sidekick
                if left_post[1] > 0 > right_post[1]:
                    # already aligned for frontkick
                    return self.action_stands_correct(connector)

                elif right_post[1] > 0:
                    # goal is on the left
                    if left_post[0] > 0 > right_post[0]:
                        # aligned for sidekick
                        return self.action_stands_correct(connector)

                elif left_post[1] < 0:
                    # goal is on the right
                    if left_post[0] < 0 < right_post[0]:
                        # aligned for sidekick
                        return self.action_stands_correct(connector)

            return self.push(AlignToGoal)

        # Stands correct
        return self.action_stands_correct(connector)

    def action_stands_correct(self, connector):
        connector.blackboard.stop_aligning()
        return self.push(KickDecisionCommon)

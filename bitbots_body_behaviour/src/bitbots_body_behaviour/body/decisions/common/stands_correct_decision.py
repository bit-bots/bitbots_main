# -*- coding:utf-8 -*-
"""
StandsCorrectDecision
^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <robocup@poppinga.xyz>

"""

from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule

import rospy
from math import atan2, sqrt
from bitbots_body_behaviour.body.actions.go_to import GoToBall, GoToRelativePosition
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
        self.ball_lost_time = connector.config["Body"]["Common"]["maxBallTime"]
        self.ball_moved_threshold = connector.config["Body"]["Common"]["ballMovedThreshold"]
        self.max_kick_attempts = connector.config["Body"]["Common"]["maxKickAttempts"]
        self.ball_lost_forward_distance = connector.config["Body"]["Common"]["ballLostForwardDistance"]

    def perform(self, connector, reevaluate=False):
        return self.action_stands_correct(connector)

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
        # The robot tries to kick several times. When the kick was successful
        # (the ball moved), he finished kicking. Else, he retries until the
        # maximum number of kicks is reached, the ball has moved or he does
        # not see the ball anymore.
        if connector.blackboard.get_kick_attempts() > 0:
            if connector.blackboard_capsule.get_kick_attempts() >= self.max_kick_attempts:
                position = (self.ball_lost_forward_distance, 0, 0)
                return self.push(GoToRelativePosition, position)

            last_ball = connector.blackboard_capsule.get_saved_ball_position()[0]
            current_ball = connector.world_model.get_ball_position_uv()
            ball_moved = sqrt((last_ball[0] - current_ball[0])**2 + (last_ball[1] - current_ball[1])**2) > self.ball_moved_threshold
            ball_lost = rospy.get_time() - connector.blackboard_capsule.get_saved_ball_position()[1] > self.ball_lost_time
            if ball_moved or ball_lost:
                connector.blackboard_capsule.reset_kick_attempts()
                return self.pop()
            else:
                connector.blackboard_capsule.save_ball_position(connector.world_model.get_ball_position_uv())
                connector.blackboard_capsule.increase_kick_attempts()
                return self.push(KickDecisionCommon)
        else:
            connector.blackboard_capsule.save_ball_position(connector.world_model.get_ball_position_uv())
            connector.blackboard_capsule.increase_kick_attempts()
            return self.push(KickDecisionCommon)

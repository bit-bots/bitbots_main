# -*- coding:utf-8 -*-
"""
Kick Decision
^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>
"""
import math

import rospy
from abstract.abstract_decision_module import AbstractDecisionModule
from body.actions.plain_walk_action import PlainWalkAction
from body.actions.throw_ball import ThrowBall
from body.actions.kick_ball import KickBall
from body.decisions.common.dynamic_kick import DynamicKick
from modell.capsules.walking_capsule import WalkingCapsule
from body.decisions.common.dynamic_kick_new import DynamicKickNew


class AbstractKickDecision(AbstractDecisionModule):
    """
    Decides wich leg he has to use to kick the ball and if he has te repositionate before kicking
    """

    def __init__(self, _):
        super(AbstractKickDecision, self).__init__()
        self.max_goal_hard_distance = rospy.get_param("Behaviour/Fieldie/maxGoalHardKickDistance")
        self.toggle_use_side_kick = rospy.get_param("Behaviour/Toggles/Fieldie/useSideKickInGame")
        self.use_dynamic_kick_toggle = rospy.get_param("Behaviour/Toggles/Fieldie/useDynamicKick")

    def perform(self, connector, reevaluate=False):

        self.action(connector, reevaluate)

    def action(self, connector, reevaluate):
        self.do_not_reevaluate()
        connector.blackboard_capsule().unset_finished_align()

        # todo check if left-right is correct
        opp_goal_left_post_v = connector.filtered_vision_capsule().get_local_goal_model_opp_goal_posts()[1][1]
        opp_goal_right_post_v = connector.filtered_vision_capsule().get_local_goal_model_opp_goal_posts()[0][1]

        if not self.toggle_use_side_kick or opp_goal_right_post_v < 0 < opp_goal_left_post_v:
            if self.use_dynamic_kick_toggle:
                # Old Version before 2015 Project - TODO: Evaluate which version is better
                # return self.push(DynamicKickNew)
                return self.push(DynamicKickNew)
            else:
                if connector.filtered_vision_capsule().get_local_goal_model_opp_goal()[0] < \
                        self.max_goal_hard_distance:
                    return self.hard_kick(connector)
                else:
                    return self.kick_normal(connector)

        elif self.toggle_use_side_kick:
            return self.kick_side_goal(connector)

    def kick_normal(self, connector):
        """
        Pushes a normal kick, depending on side of the Ball
        """
        if connector.filtered_vision_capsule().get_local_goal_model_ball()[1] <= 0:
            say("Normal Kick")
            return self.push(KickBall, init_data="R")
        else:
            say("Normal Kick")
            return self.push(KickBall, init_data="L")

    def kick_side_goal(self, connector):
        """
        Pushes a sidekick, depending on the side of the enemy goal
        """
        if connector.filtered_vision_capsule().get_local_goal_model_opp_goal()[1] > 0:
            say("Kick Ball right")
            return self.push(KickBall, init_data="SRK")
        else:
            say("Kick Ball left")
            return self.push(KickBall, init_data="SLK")

    def hard_kick(self, connector):
        """
        Pushes a hard kick, depending on side of the ball
        :param connector:
        :return:
        """

        if connector.filtered_vision_capsule().get_local_goal_model_ball()[1] <= 0:
            say("Kick Strong")
            return self.push(KickBall, init_data="RP")
        else:
            say("Kick Strong")
            return self.push(KickBall, init_data="LP")

    def get_reevaluate(self):
        return True


class KickDecisionCommon(AbstractKickDecision):
    pass


class KickDecisionThrowIn(AbstractKickDecision):
    def action(self, connector, reevaluate):

        if not connector.blackboard_capsule().get_throwin_turned():
            connector.blackboard_capsule().set_throwin_turned()
            say("Turn")
            return self.push(PlainWalkAction, [
                [WalkingCapsule.ZERO, WalkingCapsule.SLOW_ANGULAR_RIGHT, WalkingCapsule.MEDIUM_SIDEWARDS_RIGHT, 2],
                [WalkingCapsule.ZERO, WalkingCapsule.SLOW_ANGULAR_RIGHT, WalkingCapsule.ZERO, 15],
                [WalkingCapsule.ZERO, WalkingCapsule.SLOW_ANGULAR_RIGHT, WalkingCapsule.MEDIUM_SIDEWARDS_RIGHT, 15],
                [WalkingCapsule.ZERO, WalkingCapsule.SLOW_ANGULAR_RIGHT, WalkingCapsule.ZERO, 15],
                [WalkingCapsule.ZERO, WalkingCapsule.SLOW_ANGULAR_RIGHT, WalkingCapsule.MEDIUM_SIDEWARDS_RIGHT, 15],
                [WalkingCapsule.ZERO, WalkingCapsule.SLOW_ANGULAR_RIGHT, WalkingCapsule.ZERO, 15], ])

        elif not connector.blackboard_capsule().get_throwin_aligned():

            connector.blackboard_capsule().set_throwin_aligned()
            o_u, o_v = connector.blackboard_capsule().get_obstacle_position()
            o_winkel = math.degrees(math.atan2(o_u, o_v))

            say("align")
            turntime = o_winkel / 3
            say(str(turntime))
            if o_winkel > 0:
                return self.push(PlainWalkAction, [[WalkingCapsule.SLOW_BACKWARD, WalkingCapsule.MEDIUM_ANGULAR_RIGHT,
                                                    WalkingCapsule.MEDIUM_SIDEWARDS_LEFT, turntime]])
            else:
                return self.push(PlainWalkAction, [[WalkingCapsule.SLOW_BACKWARD, WalkingCapsule.MEDIUM_ANGULAR_LEFT,
                                                    WalkingCapsule.MEDIUM_SIDEWARDS_RIGHT, turntime]])

        return self.push(ThrowBall)


class KickDecisionPenaltyKick(AbstractKickDecision):  # todo not yet refactored 6.12.14.
    def action(self, connector, reevaluate):
        if connector.raw_vision_capsule().get_ball_info("v") <= 0:
            #say("Kick Strong")
            return self.push(KickBall, init_data="R")
        else:
            #say("Kick Strong")
            return self.push(KickBall, init_data="L")

# -*- coding:utf-8 -*-
"""
Hack Align
^^^^^^^^^^^^^

Versucht sich auf dümmste art zum tor auszurichten
Tries to face the goal in the dumbest possible (but working) way

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

History:
* 20.07.14: Created (Marc Bestmann)
"""
import math
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.actions.kick_ball import KickBall
from bitbots.modules.behaviour.body.actions.plain_walk_action import PlainWalkAction
from bitbots.modules.behaviour.body.actions.wait import Wait
from bitbots.modules.behaviour.modell.capsules.walking_capsule import WalkingCapsule
from bitbots.util.config import get_config
from bitbots.util.math_utils import convert_uv2polar
from bitbots.util.speaker import say
from humanoid_league_msgs.msg import HeadMode


class HackAlign(AbstractDecisionModule):
    def perform(self, connector, reevaluate=False):

        # search complete goal
        connector.blackboard_capsule().set_priorities(enemy_goal_priority=0, ball_priority=0, own_goal_priority=0,
                                                      align_priority=1510)
        # Look at the goal
        connector.blackboard.set_head_duty(HeadMode.POST_MODE)

        connector.walking_capsule().stop_walking()
        self.do_not_reevaluate()

        # waits for result from head
        if not connector.blackboard_capsule().get_complete_goal_found() and \
                not connector.blackboard_capsule().get_cant_find_complete_goal():
            return self.push(Wait, 0.1)

        # if found
        if connector.blackboard_capsule().get_complete_goal_found():
            connector.blackboard_capsule().unset_complete_goal_found()

            goal_center = connector.blackboard_capsule().get_complete_goal_center()
            goal_distance = math.sqrt(goal_center[0] ** 2 + goal_center[1] ** 2)
            goal_polar = convert_uv2polar(goal_center[0], goal_center[1])
            goal_angle = math.degrees(goal_polar[1])
            # test if own goal
            right_goal = self.test_if_right_goal(connector, goal_distance)
            goal_v = goal_center[1]
            # compute angle to move
            if not right_goal:
                connector.speaker.say("Do not shot on own goal")
                sign = self.sign(goal_angle)
                goal_angle += 180 * sign
                # ausgleich, wenns über 180 rüber geht, weil unsere polarwerte so funktionieren
                # math if it is more than 180, because our polarvalues work this way
                if goal_angle < -180:
                    goal_angle = 180 - (abs(goal_angle) - 180)
                elif goal_angle > 180:
                    goal_angle = -180 + (abs(goal_angle) - 180)

                if goal_v > 0:
                    goal_v -= 10000
                else:
                    goal_v += 10000

            if False:
                # if goodangle
                if abs(goal_angle) < 30:
                    # shoot
                    connector.blackboard_capsule().set_finished_align()
                    connector.blackboard_capsule().set_priorities(enemy_goal_priority=0, ball_priority=1000,
                                                                  own_goal_priority=0, align_priority=0)
                    connector.speaker.say("kick")
                    return self.interrupt()
                # else (useful comment...)
                else:
                    connector.speaker.say("start")
                    # feste zahl drehen
                    # turn fixed value
                    factor = 0.2
                    # chose direction
                    if self.sign(goal_angle) == 1:
                        return self.push(PlainWalkAction, [[WalkingCapsule.ZERO, WalkingCapsule.MEDIUM_ANGULAR_LEFT,
                                                            WalkingCapsule.SLOW_SIDEWARDS_RIGHT,
                                                            abs(goal_angle) * factor]])
                    else:
                        return self.push(PlainWalkAction, [[WalkingCapsule.ZERO, WalkingCapsule.MEDIUM_ANGULAR_RIGHT,
                                                            WalkingCapsule.SLOW_SIDEWARDS_LEFT,
                                                            abs(goal_angle) * factor]])
                        # warten abbuf ende der drehung nicht nötig,
                        # da wir dann einfach wieder oben anfangen und neu nach dem tor suchen

            if False:  # should work
                if goal_angle < -30:
                    connector.speaker.say("Kick Ball right")
                    return self.push(KickBall, init_data="RIGHT_SIDE_KICK")
                elif goal_angle > 30:
                    connector.speaker.say("Kick Ball left")
                    return self.push(KickBall, init_data="LEFT_SIDE_KICK")
                else:
                    if connector.raw_vision_capsule().get_ball_info("v") <= 0:
                        connector.speaker.say("Kick Strong")
                        return self.push(KickBall, init_data="RIGHT_KICK_STRONG")
                    else:
                        connector.speaker.say("Kick Strong")
                        return self.push(KickBall, init_data="LEFT_KICK_STRONG")

            if True:
                if (goal_distance > 2000 and abs(goal_angle > 30)) or (goal_distance <= 2000 and abs(goal_v) > 1000):
                    if goal_angle < 0:
                        connector.speaker.say("Kick Ball right")
                        return self.push(KickBall, init_data="RIGHT_SIDE_KICK")
                    elif goal_angle >= 0:
                        connector.speaker.say("Kick Ball left")
                        return self.push(KickBall, init_data="LEFT_SIDE_KICK")
                else:
                    if connector.raw_vision_capsule().get_ball_info("v") <= 0:
                        connector.speaker.say("Kick Strong")
                        return self.push(KickBall, init_data="RIGHT_KICK_STRONG")
                    else:
                        connector.speaker.say("Kick Strong")
                        return self.push(KickBall, init_data="LEFT_KICK_STRONG")
        else:
            connector.blackboard_capsule().unset_cant_find_complete_goal()
            # shoot, to do at least anything
            connector.blackboard_capsule().set_finished_align()
            connector.speaker.say("Cant find goal")
            connector.blackboard_capsule().set_priorities(enemy_goal_priority=0, ball_priority=1000,
                                                          own_goal_priority=0, align_priority=0)
            if connector.raw_vision_capsule().get_ball_info("v") <= 0:
                connector.speaker.say("Kick Strong")
                return self.push(KickBall, init_data="RIGHT_KICK_STRONG")
            else:
                connector.speaker.say("Kick Strong")
                return self.push(KickBall, init_data="LEFT_KICK_STRONG")

    def test_if_right_goal(self, connector, goal_distance):
        if connector.team_data_capsule().get_team_goalie_ball_position() == 0 or\
                connector.team_data_capsule().get_team_goalie_ball_position() == (
                999999, 0):
            # zwei fragen, weil ih nicht weiß, was zurückkommt, wenn ich keine daten hab
            return True  # no data from goalie

        config = get_config()['field']
        length = config['length']

        goalie_ball_distance = connector.team_data_capsule().get_goalie_ball_distance()
        delta = abs(goal_distance - goalie_ball_distance)

        if length / 2 + 500 > goalie_ball_distance > length / 2 - 500:
            # der ball liegt sehr mittig, wir treffen lieber keine entscheidung, weil die vermutlich nicht valide ist
            # the ball is in the middle, we better don't decide, because we cant be sure
            return True  # so we let him kick
        else:
            if delta < 1000:
                # falls die distanzen fast gleich sind, ist das wohl das falsche tor
                # in the case the distances are nearly equal, this is probably the wrong golal
                return False
            else:
                return True

# -*- coding:utf-8 -*-
"""
KickOffSupporter
^^^^^^^^^^^^^^^^

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

History:
* 06.01.15: Created (Marc Bestmann)
"""
import time

from bitbots_common.stackmachine.abstract_decision_module import AbstractDecisionModule


class KickOffSupporterSideDecision(AbstractDecisionModule):
    def perform(self, connector, reevaluate=False):
        say("KickOffSupporter")

        ball_u, ball_v = connector.world_model_capsule().get_ball_position_uv()
        if ball_v > 0:
            say("Left")
            return self.push(KickOffSupporter, 1)
        else:
            say("Right")
            return self.push(KickOffSupporter, -1)


class KickOffSupporter(AbstractDecisionModule):
    def __init__(self, args):
        super(KickOffSupporter, self).__init__()
        config = get_config()
        self.ignore_kick_off_time = config["Behaviour"]["Fieldie"]["KickOff"]["ignoreKickOffTime"]
        self.strategy_outdated = config["Behaviour"]["Fieldie"]["KickOff"]["strategyOutdateTime"]
        self.toggle_one_time_defender = config["Behaviour"]["Toggles"]["Fielde"]["kickOffOneTimeDefender"]
        self.start_time = time.time()
        self.direction = args
        self.walked = False

    def perform(self, connector, reevaluate=False):

        # If a certain time is up interrupt
        if time.time() - self.start_time > self.ignore_kick_off_time:
            say("Nothing is happening")
            self.interrupt()

        strategy, recive_time = connector.team_data_capsule().get_kickoff_strategy()

        if strategy != 0 and time.time() - recive_time < self.strategy_outdated:
            # only do something if the strategy was recently recived

            direction_string = "left" if strategy == -1 else "right"
            say("Strategy is: " + direction_string)
            if connector.team_data_capsule().get_kickoff_strategy() == self.direction:
                say("I am attacking")
                self.interrupt()
            else:
                if not self.walked:
                    self.walked = True
                    say("I am Defending")
                    if self.toggle_one_time_defender:
                        connector.set_duty("OneTimeDefender")
                        if self.direction == -1:
                            return self.push(PlainWalkAction,
                                             [(WalkingCapsule.FAST_BACKWARD, WalkingCapsule.ZERO, WalkingCapsule.ZERO,
                                               8),
                                              (WalkingCapsule.ZERO, WalkingCapsule.FAST_SIDEWARDS_RIGHT,
                                               WalkingCapsule.ZERO, 8)])
                        else:
                            return self.push(PlainWalkAction,
                                             [(WalkingCapsule.FAST_BACKWARD, WalkingCapsule.ZERO, WalkingCapsule.ZERO,
                                               8),
                                              (WalkingCapsule.ZERO, WalkingCapsule.FAST_SIDEWARDS_LEFT,
                                               WalkingCapsule.ZERO, 8)])
                    else:
                        connector.set_duty("Defender")
                        # the defender should automatically go to its position

        if self.walked:
            return self.interrupt()

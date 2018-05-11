# -*- coding:utf-8 -*-
"""
KickOffSupporter
^^^^^^^^^^^^^^^^

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

History:
* 06.01.15: Created (Marc Bestmann)
"""
import rospy

from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from humanoid_league_msgs.msg import Strategy


class KickOffSupporterSideDecision(AbstractDecisionModule):
    def perform(self, connector, reevaluate=False):
        connector.speaker.say("KickOffSupporter")
        ball_u, ball_v = connector.personal_model.get_ball_relative()
        if ball_v > 0:
            connector.speaker.say("Left")
            return self.push(KickOffSupporter, Strategy.SIDE_LEFT)
        else:
            connector.speaker.say("Right")
            return self.push(KickOffSupporter, Strategy.SIDE_RIGHT)


class KickOffSupporter(AbstractDecisionModule):
    def __init__(self, connector, args):
        super(KickOffSupporter, self).__init__(connector)
        self.ignore_kick_off_time = connector.config["Behaviour"]["Fieldie"]["KickOff"]["ignoreKickOffTime"]
        self.strategy_outdated = connector.config["Behaviour"]["Fieldie"]["KickOff"]["strategyOutdateTime"]
        self.toggle_one_time_defender = connector.config["Behaviour"]["Toggles"]["Fielde"]["kickOffOneTimeDefender"]
        self.start_time = rospy.get_time()
        self.direction = args
        self.walked = False

    def perform(self, connector, reevaluate=False):

        # If a certain time is up interrupt
        if rospy.get_time() - self.start_time > self.ignore_kick_off_time:
            connector.speaker.say("Nothing is happening")
            self.interrupt()

        strategy, receive_time = connector.team_data.get_kickoff_strategy()

        if strategy != Strategy.SIDE_MIDDLE and rospy.get_time() - receive_time < self.strategy_outdated:
            # only do something if the strategy was recently received

            direction_string = "left" if strategy == Strategy.SIDE_LEFT else "right"
            connector.speaker.say("Strategy is: " + direction_string)

            if strategy == self.direction:
                connector.speaker.say("I am attacking")
                self.interrupt()
            else:
                if not self.walked:
                    self.walked = True
                    connector.speaker.say("I am Defending")
                    if self.toggle_one_time_defender:
                        connector.set_duty(Strategy.ROLE_DEFENDER)
                        if self.direction == Strategy.SIDE_RIGHT:
                            # TODO: adjust distances
                            connector.pathfinding.go_to(-1, -1, 0)
                        else:
                            connector.pathfinding.go_to(-1, 1, 0)
                    else:
                        connector.set_duty(Strategy.ROLE_DEFENDER)
                        # the defender should automatically go to its position

        if self.walked:
            return self.interrupt()

"""
SearchAndConfirm
^^^^^^^^^^^^^^^^

Searches and confirms the goal or ball

"""
import time

import rospy

from bitbots_head_behaviour.actions.track_object import TrackBall, TrackGoal
from bitbots_head_behaviour.decisions.search_for_object import SearchForBall, SearchForEnemyGoal
from bitbots_common.connector.connector import HeadConnector
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class AbstractSearchAndConfirm(AbstractDecisionModule):
    def __init__(self, connector: HeadConnector, _):
        super(AbstractSearchAndConfirm, self).__init__(connector, _)
        self.set_confirmed = None
        self.get_started_confirm_time = None
        self.set_started_confirm_time = None
        self.unset_started_confirm_time = None
        self.object_seen = None
        self.object_last_seen = None
        self.fr = True

        self.fail_counter = 0
        self.confirm_time = connector.config["Head"]["Search"]["confirmTime"]
        self.track_ball_lost_time = connector.config["Head"]["Tracking"]["trackBallLost"]
        self.ball_fail_conter_max = connector.config["Head"]["Tracking"]["ballFailCounterMax"]

    def perform(self, connector: HeadConnector, reevaluate=False):
        rospy.logdebug("Ballseen: " + str(self.object_seen()))
        if time.time() - self.get_started_confirm_time() > self.confirm_time and \
                        self.get_started_confirm_time() != 0:
            self.set_confirmed()
            self.unset_started_confirm_time()
            #return self.pop()

        if self.object_seen():
            if time.time() - self.get_started_confirm_time() > self.confirm_time:
                self.set_started_confirm_time()
                self.fail_counter = 0
            self.fail_counter -= 1
            return self.track()
        else:
            self.fail_counter += 2
            if self.fail_counter >= self.ball_fail_conter_max:
                self.unset_started_confirm_time()  # stop confirming, because ball was lost
            if time.time() - self.object_last_seen() < self.track_ball_lost_time:
                return self.track()
            else:
                return self.search()

    def track(self):
        pass

    def search(self):
        pass

    def get_reevaluate(self):
        return True


class SearchAndConfirmBall(AbstractSearchAndConfirm):
    def perform(self, connector: HeadConnector, reevaluate=False):
        if self.fr:
            self.fr = False
            self.get_started_confirm_time = connector.head.get_started_confirm_ball
            self.set_started_confirm_time = connector.head.set_started_confirm_ball
            self.unset_started_confirm_time = connector.head.unset_started_confirm_ball
            self.set_confirmed = connector.head.set_confirmed_ball
            self.object_seen = connector.vision.ball_seen
            self.object_last_seen = connector.vision.ball_last_seen
        super(SearchAndConfirmBall, self).perform(connector, reevaluate)

    def track(self):
        rospy.logdebug("Push TrackBall")
        return self.push(TrackBall)

    def search(self):
        rospy.logdebug("Push SearchForBall")
        return self.push(SearchForBall)


class SearchAndConfirmEnemyGoal(AbstractSearchAndConfirm):
    def perform(self, connector: HeadConnector, reevaluate=False):
        if self.fr:
            self.fr = False
            self.get_started_confirm_time = connector.head.get_started_confirm_goal
            self.set_started_confirm_time = connector.head.set_started_confirm_goal
            self.unset_started_confirm_time = connector.head.unset_started_confirm_goal
            self.set_confirmed = connector.head.set_confirmed_goal
            self.object_seen = connector.vision.any_goal_seen
            self.object_last_seen = connector.vision.any_goalpost_last_seen
        super(SearchAndConfirmEnemyGoal, self).perform(connector, reevaluate)

    def track(self):
        return self.push(TrackGoal)

    def search(self):
        return self.push(SearchForEnemyGoal)

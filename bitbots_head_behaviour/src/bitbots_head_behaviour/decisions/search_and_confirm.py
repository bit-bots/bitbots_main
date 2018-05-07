# -*- coding:utf-8 -*-
"""
SearchAndConfirm
^^^^^^^^^^^^^^^^

Searches and confirms the goal or ball

"""
import rospy

from bitbots_head_behaviour.actions.look_at import LookAtBall, LookAtGoal
from bitbots_head_behaviour.decisions.search_for_object import SearchForBall, SearchForEnemyGoal
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class AbstractSearchAndConfirm(AbstractDecisionModule):
    def __init__(self, connector, _):
        super(AbstractSearchAndConfirm, self).__init__(connector, _)
        self.fr = True
        self.object_last_seen = None
        self.set_confirmed_time = None
        self.unset_confirmed_time = None

        self.track_ball_lost_time = connector.config["Head"]["Tracking"]["trackBallLost"]

    def perform(self, connector, reevaluate=False):
        rospy.logdebug("Ball seen: " + str(self.object_last_seen()))
        if rospy.get_time() - self.object_last_seen() < self.track_ball_lost_time:
            self.set_confirmed_time()
            return self.track()
        else:
            self.unset_confirmed_time()
            return self.search()

    def track(self):
        pass

    def search(self):
        pass

    def get_reevaluate(self):
        return True


class SearchAndConfirmBall(AbstractSearchAndConfirm):
    def perform(self, connector, reevaluate=False):
        if self.fr:
            self.fr = False
            self.set_confirmed_time = connector.head.set_confirmed_ball_time
            self.unset_confirmed_time = connector.head.unset_confirmed_ball_time
            self.object_last_seen = connector.personal_model.ball_last_seen
        super(SearchAndConfirmBall, self).perform(connector, reevaluate)

    def track(self):
        rospy.logdebug("Push LookAtBall")
        return self.push(LookAtBall)

    def search(self):
        rospy.logdebug("Push SearchForBall")
        return self.push(SearchForBall)


class SearchAndConfirmEnemyGoal(AbstractSearchAndConfirm):
    def perform(self, connector, reevaluate=False):
        if self.fr:
            # TODO: get data from world model to distinguish between own and enemy goal
            self.fr = False
            self.set_confirmed_time = connector.head.set_confirmed_goal_time
            self.unset_confirmed_time = connector.head.unset_confirmed_goal_time
            self.object_last_seen = connector.personal_model.any_goal_last_seen
        super(SearchAndConfirmEnemyGoal, self).perform(connector, reevaluate)

    def track(self):
        return self.push(LookAtGoal)

    def search(self):
        return self.push(SearchForEnemyGoal)

# -*- coding:utf-8 -*-
"""
HeadDutyDecider
^^^^^^^^^^^^^^^

"""
import time

import rospy
import math
from bitbots_head_behaviour.decisions.search_and_confirm import SearchAndConfirmBall, SearchAndConfirmEnemyGoal
from bitbots_head_behaviour.decisions.continuous_search import ContinuousSearch
from bitbots_head_behaviour.actions.head_to_pan_tilt import HeadToPanTilt
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from humanoid_league_msgs.msg import HeadMode


class HeadDutyDecider(AbstractDecisionModule):

    def __init__(self, connector, _):
        super(HeadDutyDecider, self).__init__(connector)
        self.toggle_goal_vision_tracking = connector.config["Head"]["Toggles"]["goalVisionTracking"]
        self.toggle_switch_ball_goal = connector.config["Head"]["Toggles"]["switchBallGoalSearch"]

        self.min_tilt = connector.config["Head"]["Camera"]["minTilt"]
        self.max_tilt = connector.config["Head"]["Camera"]["maxTilt"]

        self.ball_prio = 0
        self.goal_prio = 0
        self.trackjustball_aftergoal = False

    def perform(self, connector, reevaluate=False):

        # set priorities
        if connector.personal_model.ball_seen():
            self.ball_prio = max(0, self.ball_prio - 3)
        else:
            self.ball_prio = min(120, self.ball_prio + 5)

        if connector.personal_model.any_goal_seen():
            self.goal_prio = max(0, self.goal_prio - 2)
        else:
            self.goal_prio = min(100, self.goal_prio + 3)

        rospy.logdebug("GoalPrio" + str(self.goal_prio))
        rospy.logdebug("BallPrio" + str(self.ball_prio))
        rospy.logdebug("BallLastConfirmed" + str(rospy.get_time() - connector.head.get_confirmed_ball_time()))

        head_mode = connector.head.get_headmode()
        if head_mode == "":
            return self.interrupt()

        if head_mode == HeadMode.DONT_MOVE:
            return self.interrupt()

        if head_mode == HeadMode.BALL_MODE:
            return self.push(SearchAndConfirmBall)

        if head_mode == HeadMode.POST_MODE:
            return self.push(SearchAndConfirmEnemyGoal)

        if head_mode == HeadMode.BALL_GOAL_TRACKING:
            rospy.logdebug("TrackbothTime", rospy.get_time())
            if rospy.get_time() - connector.head.get_confirmed_ball_time() > 5:
                return self.push(SearchAndConfirmBall)

            # ball long enough seen
            elif rospy.get_time() - connector.head.get_confirmed_goal_time() > 6:
                return self.push(SearchAndConfirmEnemyGoal)

            elif self.trackjustball_aftergoal:
                return self.push(SearchAndConfirmBall)

        if head_mode == HeadMode.FIELD_FEATURES:
            return self.push(ContinuousSearch)

        if head_mode == HeadMode.LOOK_DOWN:
            pan_tilt = 0, self.min_tilt
            return self.push(HeadToPanTilt, pan_tilt)

        if head_mode == HeadMode.LOOK_FORWARD:
            pan_tilt = 0, -12
            return self.push(HeadToPanTilt, pan_tilt)

        if head_mode == HeadMode.LOOK_UP:
            pan_tilt = 0, self.max_tilt
            return self.push(HeadToPanTilt, pan_tilt)

        if self.toggle_switch_ball_goal:
            rospy.logdebug("Headdoes", "Priorities")
            if self.ball_prio >= self.goal_prio:
                return self.push(SearchAndConfirmBall)
            else:
                return self.push(SearchAndConfirmEnemyGoal)

        # Default Head Behaviour
        return 'defaultSearch'

    def get_reevaluate(self):
        return True

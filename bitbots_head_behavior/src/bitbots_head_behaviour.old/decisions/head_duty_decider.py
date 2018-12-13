# -*- coding:utf-8 -*-
"""
HeadDutyDecider
^^^^^^^^^^^^^^^

"""
import rospy
from bitbots_head_behaviour.decisions.search_and_confirm import SearchAndConfirmBall, SearchAndConfirmEnemyGoal
from bitbots_head_behaviour.decisions.continuous_search import ContinuousSearch
from bitbots_head_behaviour.actions.look_at import LookAtRelativePoint
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from humanoid_league_msgs.msg import HeadMode


class HeadDutyDecider(AbstractDecisionElement):

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
        if connector.world_model.ball_seen():
            self.ball_prio = max(0, self.ball_prio - 3)
        else:
            self.ball_prio = min(120, self.ball_prio + 5)

        if connector.world_model.any_goal_seen():
            self.goal_prio = max(0, self.goal_prio - 2)
        else:
            self.goal_prio = min(100, self.goal_prio + 3)

        rospy.logdebug("GoalPrio" + str(self.goal_prio))
        rospy.logdebug("BallPrio" + str(self.ball_prio))
        rospy.logdebug("BallLastConfirmed" + str(rospy.get_time() - connector.head.get_confirmed_ball_time()))

        head_mode = HeadMode.BALL_MODE #connector.head.get_headmode()
        if head_mode == "":
            return self.interrupt()

        if head_mode == HeadMode.DONT_MOVE:
            return self.interrupt()

        if head_mode == HeadMode.BALL_MODE:
            self.publish_debug_data("head_mode","BALL_MODE")
            return "BALL", None

        if head_mode == HeadMode.POST_MODE:
            self.publish_debug_data("head_mode","POST_MODE")
            return "POST", None

        if head_mode == HeadMode.BALL_GOAL_TRACKING:
            self.publish_debug_data("head_mode","BALL_GOAL_TRACKING")

            rospy.logdebug("TrackbothTime", rospy.get_time())
            if rospy.get_time() - connector.head.get_confirmed_ball_time() > 5:
                return "BOTH", None

            # ball long enough seen
            elif rospy.get_time() - connector.head.get_confirmed_goal_time() > 6:
                return "EnemyGoal", None

            elif self.trackjustball_aftergoal:
                self.publish_debug_data("trackjustball_aftergoal",True)
                return "AfterGoal", None

        if head_mode == HeadMode.FIELD_FEATURES:
            self.publish_debug_data("head_mode","FIELD_FEATURES")
            return "FIELD", None

        if head_mode == HeadMode.LOOK_DOWN:
            self.publish_debug_data("head_mode","LOOK_DOWN")
            return "Position", (0.05, 0, 0)

        if head_mode == HeadMode.LOOK_FORWARD:
            self.publish_debug_data("head_mode","LOOK_FORWARD")
            return "Position" ,(100, 0, 0)

        if head_mode == HeadMode.LOOK_UP:
            self.publish_debug_data("head_mode","LOOK_UP")
            return "Position", (0, 0, 10)

        self.publish_debug_data("toggle_switch_ball_goal",self.toggle_switch_ball_goal)
        if self.toggle_switch_ball_goal:
            rospy.logdebug("Headdoes", "Priorities")
            self.publish_debug_data("ball_prio",self.ball_prio)
            self.publish_debug_data("goal_prio",self.goal_prio)

            if self.ball_prio >= self.goal_prio:
                return "BALL", None
            else:
                return "BALL", None

        return "Interrupt", None

    def get_reevaluate(self):
        return True

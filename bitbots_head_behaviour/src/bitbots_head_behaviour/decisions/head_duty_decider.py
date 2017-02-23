"""
HeadDutyDecider
^^^^^^^^^^^^^^^

"""
import time

import rospy
from bitbots_common.connector.connector import HeadConnector
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from humanoid_league_msgs.msg import HeadMode


class HeadDutyDecider(AbstractDecisionModule):

    def __init__(self, connector: HeadConnector, _):
        super().__init__(connector)
        self.toggle_goal_vision_tracking = connector.config["Toggles"]["goalVisionTracking"]
        self.toggle_switch_ball_goal = connector.config["Toggles"]["switchBallGoalSearch"]
        self.confirm_time = connector.config["Search"]["confirmTime"]

        self.last_confirmd_goal = 0
        self.fail_goal_counter = 0
        self.ball_prio = 0
        self.goal_prio = 0
        self.trackjustball_aftergoal = False

    def perform(self, connector: HeadConnector, reevaluate: bool=False):

        # set priorities
        if connector.vision.ball_seen():
            self.ball_prio = max(0, self.ball_prio - 3)
        else:
            self.ball_prio = min(120, self.ball_prio + 5)

        if connector.vision.any_goal_seen():
            self.goal_prio = max(0, self.goal_prio - 2)
        else:
            self.goal_prio = min(100, self.goal_prio + 3)

        rospy.loginfo("GoalPrio", self.goal_prio)
        rospy.loginfo("BallPrio", self.ball_prio)
        rospy.loginfo("BallLastConfirmed", time.time() - connector.head.get_confirmed_ball())
        rospy.loginfo("BallLastStratedconfirm", time.time() - connector.head.get_started_confirm_ball())

        head_mode = connector.head.get_headmode()
        if head_mode == "":
            return 'nothing'

        if head_mode == HeadMode.BALL_MODE:
            return 'trackBall'

        if head_mode == HeadMode.POLE_MODE:
            return 'trackGoal'

        if head_mode == HeadMode.BALL_GOAL_TRACKING:
            rospy.loginfo("TrackbothTime", time.time())
            if time.time() - connector.head.get_confirmed_ball() > 5:
                return 'trackBothBall'

            # ball long enough seen
            elif time.time() - connector.head.get_confirmed_goal() > 6:
                return 'trackBothGoal'

            elif self.trackjustball_aftergoal:
                return 'trackBothElse'

        if self.toggle_switch_ball_goal:
            rospy.loginfo("Headdoes", "Priorities")
            if self.ball_prio >= self.goal_prio:
                return 'searchAndConfirmBall'
            else:
                return 'searchAndConfirmGoal'

        # Default Head Behaviour
        return 'defaultSearch'

    def get_reevaluate(self):
        return True

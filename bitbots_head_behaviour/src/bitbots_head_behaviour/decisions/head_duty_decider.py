"""
HeadDutyDecider
^^^^^^^^^^^^^^^

"""
import rospy
import time


from bitbots_head_behaviour.head_connector import HeadConnector
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class HeadDutyDecider(AbstractDecisionModule):

    def __init__(self, connector: HeadConnector,
                 outcomes=('nothing', 'trackBall', 'trackGoal', 'trackBothBall', 'trackBothGoal', 'trackBothElse',
                           'searchAndConfirmBall', 'searchAndConfirmGoal', 'defaultSearch')):
        super().__init__(connector)
        self.toggle_goal_vison_tracking = connector.config["Toggles"]["goalVisionTracking"]
        self.toggle_switch_ball_goal = connector.config["Toggles"]["switchBallGoalSearch"]
        self.confirm_time = connector.config["Search"]["confirmTime"]

        self.last_confirmd_goal = 0
        self.fail_goal_counter = 0
        self.ball_prio = 0
        self.goal_prio = 0
        self.trackjustball_aftergoal = False

    def execute(self, connector: HeadConnector):

        # set priorities
        if connector.raw_vision_capsule().ball_seen():
            self.ball_prio = max(0, self.ball_prio - 3)
        else:
            self.ball_prio = min(120, self.ball_prio + 5)

        if connector.raw_vision_capsule().any_goal_seen():
            self.goal_prio = max(0, self.goal_prio - 2)
        else:
            self.goal_prio = min(100, self.goal_prio + 3)

        rospy.loginfo("GoalPrio", self.goal_prio)
        rospy.loginfo("BallPrio", self.ball_prio)
        rospy.loginfo("BallLastCOnfirmed", time.time() - connector.blackboard_capsule().get_confirmed_ball())
        rospy.loginfo("BallLastStratedconfirm", time.time() - connector.blackboard_capsule().get_started_confirm_ball())

        head_mode = connector.get_headmode()
        if head_mode == DONT_MOVE:
            return 'nothing'

        if head_mode == BALL_MODE:
            return 'trackBall'

        if head_mode == POLE_MODE:
            return 'trackGoal'

        if head_mode == BALL_GOAL_TRACKING:
            rospy.loginfo("TrackbothTime", time.time())
            if time.time() - connector.get_confirmed_ball() > 5:
                return 'trackBothBall'

            # ball long enough seen
            elif time.time() - connector.get_confirmed_goal() > 6:
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

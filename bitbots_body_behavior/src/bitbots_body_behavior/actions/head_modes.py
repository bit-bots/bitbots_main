from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode


class LookAtBall(AbstractActionElement):
    """Search for Ball and track it if found"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.BALL_MODE)
        return self.pop()


class LookAtGoalPosts(AbstractActionElement):
    """Search for goal posts, mainly to locate the robot on the field"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.POST_MODE)
        return self.pop()


class LookAtBallAndGoalPosts(AbstractActionElement):
    """Track ball and goal by constantly switching between both"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.BALL_GOAL_TRACKING)
        return self.pop()


class LookAtFieldFeatures(AbstractActionElement):
    """Look generally for all features on the field (ball, goals, corners, center point)"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.FIELD_FEATURES)
        return self.pop()


class LookAtNonFieldFeatures(AbstractActionElement):
    """Look for features outside of the field (perimeter advertising, walls, etc).
    Can be used for localization using features on the ceiling."""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.NON_FIELD_FEATURES)
        return self.pop()


class LookDown(AbstractActionElement):
    """Simply look down to its feet."""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.LOOK_DOWN)
        return self.pop()


class LookForward(AbstractActionElement):
    """Simply look directly forward"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.LOOK_FORWARD)
        return self.pop()


class DontMoveHead(AbstractActionElement):
    """Don't move the head"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.DONT_MOVE)
        return self.pop()


class LookUp(AbstractActionElement):
    """Look to the ceiling, for example for visual compass"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.LOOK_UP)
        return self.pop()


class HeadRecordVisualCompass(AbstractActionElement):
    """Record ground truth for the visual compass"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.RECORD_VISUAL_COMPASS)
        return self.pop()


class LookAtBallPenalty(AbstractActionElement):
    """Ball Mode adapted for Penalty Kick"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.BALL_MODE_PENALTY)
        return self.pop()


class LookForVisualCompassFeatures(AbstractActionElement):
    """Look for visual compass features"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.VISUAL_COMPASS_FEATURES)
        return self.pop()

class LookAtFront(AbstractActionElement):
    """Search in the front of the robot"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.LOOK_FRONT)
        return self.pop()

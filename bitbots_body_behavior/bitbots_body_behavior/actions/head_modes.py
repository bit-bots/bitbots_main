from bitbots_blackboard.blackboard import BodyBlackboard

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode


class AbstractHeadModeElement(AbstractActionElement):
    """Abstract class used for type hinting"""
    blackboard: BodyBlackboard

class LookAtBall(AbstractHeadModeElement):
    """Search for Ball and track it if found"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.BALL_MODE)
        return self.pop()


class LookAtGoalPosts(AbstractHeadModeElement):
    """Search for goal posts, mainly to locate the robot on the field"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.POST_MODE)
        return self.pop()


class LookAtBallAndGoalPosts(AbstractHeadModeElement):
    """Track ball and goal by constantly switching between both"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.BALL_GOAL_TRACKING)
        return self.pop()


class LookAtFieldFeatures(AbstractHeadModeElement):
    """Look generally for all features on the field (ball, goals, corners, center point)"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.FIELD_FEATURES)
        return self.pop()


class LookAtNonFieldFeatures(AbstractHeadModeElement):
    """Look for features outside of the field (perimeter advertising, walls, etc).
    Can be used for localization using features on the ceiling."""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.NON_FIELD_FEATURES)
        return self.pop()


class LookDown(AbstractHeadModeElement):
    """Simply look down to its feet."""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.LOOK_DOWN)
        return self.pop()


class LookForward(AbstractHeadModeElement):
    """Simply look directly forward"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.LOOK_FORWARD)
        return self.pop()


class DontMoveHead(AbstractHeadModeElement):
    """Don't move the head"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.DONT_MOVE)
        return self.pop()


class LookUp(AbstractHeadModeElement):
    """Look to the ceiling, for example for visual compass"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.LOOK_UP)
        return self.pop()


class HeadRecordVisualCompass(AbstractHeadModeElement):
    """Record ground truth for the visual compass"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.RECORD_VISUAL_COMPASS)
        return self.pop()


class LookAtBallPenalty(AbstractHeadModeElement):
    """Ball Mode adapted for Penalty Kick"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.BALL_MODE_PENALTY)
        return self.pop()


class LookForVisualCompassFeatures(AbstractHeadModeElement):
    """Look for visual compass features"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.VISUAL_COMPASS_FEATURES)
        return self.pop()

class LookAtFront(AbstractHeadModeElement):
    """Search in the front of the robot"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.LOOK_FRONT)
        return self.pop()

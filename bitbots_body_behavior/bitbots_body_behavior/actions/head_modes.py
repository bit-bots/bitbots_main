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


class LookAtFieldFeatures(AbstractHeadModeElement):
    """Look generally for all features on the field (ball, goals, corners, center point)"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.FIELD_FEATURES)
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


class LookAtBallPenalty(AbstractHeadModeElement):
    """Ball Mode adapted for Penalty Kick"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.BALL_MODE_PENALTY)
        return self.pop()

class LookAtFront(AbstractHeadModeElement):
    """Search in front of the robot"""
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.LOOK_FRONT)
        return self.pop()

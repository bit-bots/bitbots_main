from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard
from geometry_msgs.msg import Twist

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import RobotControlState


class StopWalking(AbstractActionElement):
    """
    Stop the walking
    """
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard: HcmBlackboard

    def perform(self, reevaluate=False):
        msg = Twist()
        msg.angular.x = -1.0
        self.blackboard.walk_pub.publish(msg)
        self.pop()


class ForceStopWalking(AbstractActionElement):
    """
    Stop the walking and set the state to penalty
    """
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard: HcmBlackboard

    def perform(self, reevaluate=False):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        msg.angular.x = -1.0
        self.blackboard.walk_pub.publish(msg)
        self.blackboard.current_state = RobotControlState.PENALTY
        # We can pop immediately because the state is PENALTY on no walking messages will be passed
        self.pop()

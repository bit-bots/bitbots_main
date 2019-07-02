from bitbots_hcm.hcm_dsd.hcm_blackboard import STATE_PENALTY
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from geometry_msgs.msg import Twist


class StopWalking(AbstractActionElement):
    """
    Stop the walking
    """

    def perform(self, reevaluate=False):
        if self.blackboard.current_time.to_sec() - self.blackboard.last_walking_goal_time.to_sec() < 0.1:
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.angular.z = 0
            self.blackboard.walk_pub.publish(msg)
        else:
            self.pop()


class PenaltyStopWalking(AbstractActionElement):
    """
    Stop the walking and set the state to penalty
    """
    def perform(self, reevaluate=False):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.angular.z = 0
        self.blackboard.walk_pub.publish(msg)
        self.blackboard.current_state = STATE_PENALTY
        # We can pop immediately because the state is PENALTY on no walking messages will be passed
        self.pop()

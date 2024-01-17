from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement
from geometry_msgs.msg import Twist


class StopWalking(AbstractHCMActionElement):
    """
    Stop the walking
    """
    def perform(self, reevaluate=False):
        msg = Twist()
        msg.angular.x = -1.0
        self.blackboard.walk_pub.publish(msg)
        self.pop()

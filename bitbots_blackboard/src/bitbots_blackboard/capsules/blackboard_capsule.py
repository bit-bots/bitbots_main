"""
BehaviourBlackboardCapsule
^^^^^^^^^^^^^^^^^^^^^^^^^^
"""

import math
import rosparam
import rospy
import tf2_ros as tf2
from humanoid_league_msgs.msg import RobotControlState

from humanoid_league_msgs.msg import HeadMode


class BlackboardCapsule:
    def __init__(self):
        self.my_data = {}
        self.head_pub = None  # type: rospy.Publisher
        self.duty = rospy.get_param('role')  # TODO: adapt to Leo's script
        self.state = None  # type: RobotControlState

        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2.TransformListener(self.tf_buffer)

    #####################
    # ## Tracking Part ##
    #####################

    def set_head_duty(self, head_duty):
        head_duty_msg = HeadMode()
        head_duty_msg.headMode = head_duty
        self.head_pub.publish(head_duty_msg)

    ###################
    # ## Robot state ##
    ###################

    def robot_state_callback(self, msg):
        self.state = msg

    def is_currently_walking(self):
        if self.state:
            return self.state.state == RobotControlState.WALKING
        else:
            return False

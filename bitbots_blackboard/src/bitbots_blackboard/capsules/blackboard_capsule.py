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
        self.timers = dict()

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

    #############
    # ## Timer ##
    #############

    def start_timer(self, timer_name, duration_secs):
        """
        Starts a timer
        :param timer_name: Name of the timer
        :param duration_secs: Duration of the timer in seconds
        :return: None
        """
        self.timers[timer_name] = rospy.Time.now() + rospy.Duration.from_sec(int(duration_secs))

    def end_timer(self, timer_name):
        """
        Ends a timer
        :param timer_name: Name of the timer
        :return: None
        """
        self.timers[timer_name] = rospy.Time.now()

    def timer_running(self, timer_name):
        """
        Returns whether the timer is running
        :param timer_name: Name of the timer
        :return: Whether the timer is running. False if the timer doesn't exist.
        """
        if timer_name not in self.timers:
            return False
        return rospy.Time.now() < self.timers[timer_name]

    def timer_remaining(self, timer_name):
        """
        Returns whether the timer is running
        :param timer_name: Name of the timer
        :return: Whether the timer is running. False if the timer doesn't exist.
        """

        if timer_name not in self.timers:
            return -1
        return rospy.Time.now() - self.timers[timer_name]

    def timer_ended(self, timer_name):
        """
        Returns whether the timer has ended
        :param timer_name: Name of the timer
        :return: Whether the timer has ended. Also true, if timer doesn't exist.
        """
        if timer_name not in self.timers:
            return True  # Don't wait for a non-existing Timer
        return rospy.Time.now() > self.timers[timer_name]


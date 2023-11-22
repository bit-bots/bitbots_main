"""
MiscCapsule
^^^^^^^^^^^

Capsule for miscellaneous things that don't fit anywhere else.
"""
from typing import Optional

from bitbots_msgs.msg import HeadMode, RobotControlState
from bitbots_utils.utils import get_parameters_from_other_node
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool


class MiscCapsule:
    def __init__(self, node: Node):
        self.node = node
        self.head_pub = node.create_publisher(HeadMode, "head_mode", 10)

        # Config
        gamestate_settings = get_parameters_from_other_node(
            self.node, "parameter_blackboard", ["bot_id", "position_number"]
        )

        self.position_number: int = gamestate_settings["position_number"]
        self.bot_id: int = gamestate_settings["bot_id"]

        self.robot_control_state: Optional[RobotControlState] = None
        self.timers = dict()

        self.hcm_deactivate_pub = node.create_publisher(Bool, "hcm_deactivate", 1)

    #####################
    # ## Tracking Part ##
    #####################

    def set_head_duty(self, head_duty: int):
        head_duty_msg = HeadMode()
        head_duty_msg.head_mode = head_duty
        self.head_pub.publish(head_duty_msg)

    ###################
    # ## Robot state ##
    ###################

    def robot_state_callback(self, msg: RobotControlState):
        self.robot_control_state = msg

    def is_currently_walking(self) -> bool:
        return self.robot_control_state is not None and self.robot_control_state.state == RobotControlState.WALKING

    #############
    # ## Timer ##
    #############

    def start_timer(self, timer_name: str, duration_secs: int):
        """
        Starts a timer
        :param timer_name: Name of the timer
        :param duration_secs: Duration of the timer in seconds
        :return: None
        """
        self.timers[timer_name] = self.node.get_clock().now() + Duration(seconds=duration_secs)

    def end_timer(self, timer_name: str):
        """
        Ends a timer
        :param timer_name: Name of the timer
        :return: None
        """
        self.timers[timer_name] = self.node.get_clock().now()

    def timer_running(self, timer_name: str) -> bool:
        """
        Returns whether the timer is running
        :param timer_name: Name of the timer
        :return: Whether the timer is running. False if the timer doesn't exist.
        """
        if timer_name not in self.timers:
            return False
        return self.node.get_clock().now() < self.timers[timer_name]

    def timer_remaining(self, timer_name: str) -> int:
        """
        Returns how much seconds are remaining on the Timer
        :param timer_name: Name of the timer
        :return: The number of seconds left on the timer. -1 if the timer doesn't exist.
        """

        if timer_name not in self.timers:
            return -1
        return (self.timers[timer_name] - self.node.get_clock().now()).to_sec()

    def timer_ended(self, timer_name: str) -> bool:
        """
        Returns whether the timer has ended
        :param timer_name: Name of the timer
        :return: Whether the timer has ended. Also true, if timer doesn't exist.
        """
        if timer_name not in self.timers:
            return True  # Don't wait for a non-existing Timer
        return self.node.get_clock().now() > self.timers[timer_name]

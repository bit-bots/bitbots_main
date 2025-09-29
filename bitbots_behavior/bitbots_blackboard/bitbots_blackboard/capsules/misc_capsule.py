from typing import Literal, Optional, TypeAlias

from bitbots_utils.utils import RobotNotConfiguredError, get_parameters_from_other_node
from rclpy.duration import Duration
from std_msgs.msg import Bool

from bitbots_blackboard.capsules import AbstractBlackboardCapsule
from bitbots_msgs.msg import Audio, HeadMode, RobotControlState

THeadMode: TypeAlias = Literal[  # type: ignore[valid-type]
    HeadMode.TRACK_BALL,
    HeadMode.SEARCH_FIELD_FEATURES,
    HeadMode.LOOK_FORWARD,
    HeadMode.DONT_MOVE,
    HeadMode.SEARCH_BALL_PENALTY,
    HeadMode.SEARCH_FRONT,
]


class MiscCapsule(AbstractBlackboardCapsule):
    """Capsule for miscellaneous functions."""

    def __init__(self, node, blackboard):
        super().__init__(node, blackboard)
        self.head_pub = self._node.create_publisher(HeadMode, "head_mode", 10)
        self.speak_pub = self._node.create_publisher(Audio, "speak", 10)

        # Config
        gamestate_settings = get_parameters_from_other_node(
            self._node, "parameter_blackboard", ["bot_id", "position_number"]
        )

        if any(param_val is None for param_val in gamestate_settings.values()):
            error_text = """
The robot is not configured properly or the parameter_blackboard is not found.
It is likely that the robot was not configured when you syncronised your clean code onto the robot.
Run the deploy_robots script with the -c option to configure the robot and set parameters like the robot id and its role."""
            self._node.get_logger().fatal(error_text)
            raise RobotNotConfiguredError(error_text)

        self.position_number: int = gamestate_settings["position_number"]
        self.bot_id: int = gamestate_settings["bot_id"]

        self.robot_control_state: Optional[RobotControlState] = None
        self.timers = dict()

        self.hcm_deactivate_pub = self._node.create_publisher(Bool, "hcm_deactivate", 1)

    #####################
    # ## Tracking Part ##
    #####################

    def set_head_duty(
        self,
        head_duty: THeadMode,
    ) -> None:
        head_duty_msg = HeadMode()
        head_duty_msg.head_mode = head_duty
        self.head_pub.publish(head_duty_msg)

    ###################
    # ## Robot state ##
    ###################

    def robot_state_callback(self, msg: RobotControlState) -> None:
        self.robot_control_state = msg

    def is_currently_walking(self) -> bool:
        return self.robot_control_state is not None and self.robot_control_state.state == RobotControlState.WALKING

    #############
    # ## Timer ##
    #############

    def start_timer(self, timer_name: str, duration_secs: int) -> None:
        """
        Starts a timer
        :param timer_name: Name of the timer
        :param duration_secs: Duration of the timer in seconds
        :return: None
        """
        self.timers[timer_name] = self._node.get_clock().now() + Duration(seconds=duration_secs)

    def end_timer(self, timer_name: str) -> None:
        """
        Ends a timer
        :param timer_name: Name of the timer
        :return: None
        """
        self.timers[timer_name] = self._node.get_clock().now()

    def timer_running(self, timer_name: str) -> bool:
        """
        Returns whether the timer is running
        :param timer_name: Name of the timer
        :return: Whether the timer is running. False if the timer doesn't exist.
        """
        if timer_name not in self.timers:
            return False
        return self._node.get_clock().now() < self.timers[timer_name]

    def timer_remaining(self, timer_name: str) -> int:
        """
        Returns how much seconds are remaining on the Timer
        :param timer_name: Name of the timer
        :return: The number of seconds left on the timer. -1 if the timer doesn't exist.
        """

        if timer_name not in self.timers:
            return -1
        return (self.timers[timer_name] - self._node.get_clock().now()).to_sec()

    def timer_ended(self, timer_name: str) -> bool:
        """
        Returns whether the timer has ended
        :param timer_name: Name of the timer
        :return: Whether the timer has ended. Also true, if timer doesn't exist.
        """
        if timer_name not in self.timers:
            return True  # Don't wait for a non-existing Timer
        return self._node.get_clock().now() > self.timers[timer_name]

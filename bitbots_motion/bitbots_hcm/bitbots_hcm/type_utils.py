from typing import Literal, TypeAlias

from bitbots_msgs.msg import RobotControlState

T_RobotControlState: TypeAlias = Literal[  # type: ignore[valid-type]
    RobotControlState.CONTROLLABLE,
    RobotControlState.FALLING,
    RobotControlState.FALLEN,
    RobotControlState.GETTING_UP,
    RobotControlState.ANIMATION_RUNNING,
    RobotControlState.STARTUP,
    RobotControlState.SHUTDOWN,
    RobotControlState.PENALTY,
    RobotControlState.RECORD,
    RobotControlState.WALKING,
    RobotControlState.MOTOR_OFF,
    RobotControlState.HCM_OFF,
    RobotControlState.HARDWARE_PROBLEM,
    RobotControlState.PICKED_UP,
    RobotControlState.KICKING,
]

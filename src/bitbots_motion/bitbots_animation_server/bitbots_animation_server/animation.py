from typing import Optional


class Keyframe:
    """
    A pose which the robot reaches at :attr:`duration` seconds in the future.
    It pauses there for :attr:`pause` seconds before moving to the next keyframe.

    :param goals: The goal positions for each joint.
    :param torque: The torque for each joint.
    :param kp: The proportional gain for each joint. Joints without an entry use the default gain.
    :param kd: The derivative gain for each joint. Joints without an entry use the default gain.
    :param duration: The time in seconds until the robot reaches the goal.
    :param pause: The time in seconds the robot pauses at the goal.
    :param stabilization_functions: A string expression for each joint to stabilize the joint using e.g. IMU data.
    """

    def __init__(
        self,
        goals,
        torque: Optional[dict[str, float]] = None,
        kp: Optional[dict[str, float]] = None,
        kd: Optional[dict[str, float]] = None,
        duration: float = 1.0,
        pause: float = 0.0,
        stabilization_functions: Optional[dict[str, str]] = None,
    ):
        self.duration = float(duration)
        self.pause = float(pause)
        self.goals = goals
        self.torque = torque or {}
        self.kp = kp or {}
        self.kd = kd or {}
        self.stabilization_functions: dict[str, str] = stabilization_functions or {}


class Animation:
    """
    An animation is constructed by an array of goal positions (:class:`Keyframe`).
    Between two keyframes, the goal positions are interpolated.
    """

    def __init__(self, name: str, keyframes: list[Keyframe]):
        self.name: str = name
        self.keyframes: list[Keyframe] = keyframes


def parse(info: dict) -> Animation:
    """
    This method is parsing an animation from a :class:`dict`
    instance *info*, as created by :func:`as_dict`.
    """
    anim = Animation(info["name"], [])

    keyframes = info.get("keyframes", [])
    anim.keyframes = [
        Keyframe(
            k.get("goals", {}),
            k.get("torque", {}),
            k.get("kp", {}),
            k.get("kd", {}),
            k.get("duration", 1),
            k.get("pause", 0),
            k.get("stabilization_functions", {}),
        )
        for k in keyframes
    ]

    return anim

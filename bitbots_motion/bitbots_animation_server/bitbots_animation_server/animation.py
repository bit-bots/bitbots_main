from typing import Optional


class Keyframe:
    """
    A pose which the robot reaches at :attr:`duration` seconds in the future.
    It pauses there for :attr:`pause` seconds before moving to the next keyframe.

    :param goals: The goal positions for each joint.
    :param torque: The torque for each joint.
    :param duration: The time in seconds until the robot reaches the goal.
    :param pause: The time in seconds the robot pauses at the goal.
    :param stabelization_functions: A string expression for each joint to stabilize the joint using e.g. IMU data.
    """

    def __init__(
        self,
        goals,
        torque: Optional[dict[str, float]] = None,
        duration: float = 1.0,
        pause: float = 0.0,
        stabelization_functions: Optional[dict[str, str]] = None,
    ):
        self.duration = float(duration)
        self.pause = float(pause)
        self.goals = goals
        self.torque = torque or {}
        self.stabelization_functions: dict[str, str] = stabelization_functions or {}


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
    anim = Animation(info["name"], ())

    keyframes = info.get("keyframes", ())
    anim.keyframes = [
        Keyframe(
            k.get("goals", {}),
            k.get("torque", {}),
            k.get("duration", 1),
            k.get("pause", 0),
            k.get("stabelization_functions", {}),
        )
        for k in keyframes
    ]

    return anim


def as_dict(anim: Animation) -> dict:
    """
    Convert an animation to builtin python types to
    make it serializable to formats like ``json``.
    """
    return {
        "name": anim.name,
        "interpolators": {n: ip.__name__ for n, ip in anim.interpolators},
        "keyframes": [
            {
                "duration": k.duration,
                "pause": k.pause,
                "goals": k.goals,
                "torque": k.torque,
                "stabelization_functions": k.stabelization_functions,
            }
            for k in anim.keyframes
        ],
    }

class Keyframe:
    """
    A pose which the robot reaches at :attr:`duration` seconds in the future.
    """

    def __init__(self, goals, torque=None, duration=1.0, pause=0.0, p=None, stabelization_functions=None):
        self.duration = float(duration)
        self.pause = float(pause)
        self.goals = goals
        self.torque = torque or {}
        self.p = p or {}
        self.stabelization_functions: dict[str, str] = stabelization_functions or {}


class Animation:
    """
    An animation is constructed by an array of goal positions (:class:`Keyframe`).
    Between two keyframes, the goal positions are interpolated by an  :class:`Interpolator`.
    """

    def __init__(self, name, keyframes, default_interpolator=None):
        self.name: str = name
        self.keyframes: list[Keyframe] = keyframes
        self.default_interpolator = default_interpolator


def parse(info):
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
            k.get("p", {}),
            k.get("stabelization_functions", {}),
        )
        for k in keyframes
    ]

    return anim


def as_dict(anim: Animation):
    """
    Convert an animation to builtin python types to
    make it serializable to formats like ``json``.
    """
    return {
        "name": anim.name,
        "default_interpolator": anim.default_interpolator.__name__,
        "interpolators": {n: ip.__name__ for n, ip in anim.interpolators},
        "keyframes": [
            {
                "duration": k.duration,
                "pause": k.pause,
                "goals": k.goals,
                "torque": k.torque,
                "p": k.p,
                "stabelization_functions": k.stabelization_functions,
            }
            for k in anim.keyframes
        ],
    }

# -*- coding:utf-8 -*-

import time

import math
import rospy

class Keyframe:
    '''
    A pose which the robot reaches at :attr:`duration` seconds in the future.
    '''

    def __init__(self, goals, torque={}, duration=1.0, pause=0.0, p={}):
        self.duration = float(duration)
        self.pause = float(pause)
        self.goals = goals
        self.torque = torque
        self.p = p


class Animation:
    '''
    An animation is constructed by an array of goal positions (:class:`Keyframe`).
    Between two keyframes, the goal positions are interpolated by an  :class:`Interpolator`.
    '''

    def __init__(self, name, keyframes, default_interpolator=None):
        self.name = name
        self.keyframes = keyframes
        self.default_interpolator = default_interpolator


def parse(info):
    '''
    This method is parsing an animation from a :class:`dict`
    instance *info*, as created by :func:`as_dict`.
    '''
    anim = Animation(info["name"], ())

    keyframes = info.get("keyframes", ())
    anim.keyframes = [Keyframe(k.get('goals', {}), k.get('torque', {}), k.get('duration', 1), k.get('pause', 0), k.get('p', {})) for k in
                      keyframes]

    return anim


def as_dict(anim):
    '''
    Convert an animation to builtin python types to
    make it serializable to formats like ``json``.
    '''
    return {
        "name": anim.name,
        "default_interpolator": anim.default_interpolator.__name__,
        "interpolators": {n: ip.__name__ for n, ip in anim.interpolators},
        "keyframes": [{
                          "duration": k.duration,
                          "pause": k.pause,
                          "goals": k.goals,
                          "torque": k.torque,
                          "p": k.p
                      } for k in anim.keyframes]
    }

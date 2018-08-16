# -*- coding:utf-8 -*-

import time

import math
import rospy

#from bitbots_common.pose.pypose import PyJoint as Joint

class Keyframe:
    '''
    A pose which the robot reaches at :attr:`duration` seconds in the future.
    '''

    def __init__(self, goals, duration=1.0, pause=0.0, p={}):
        self.duration = float(duration)
        self.pause = float(pause)
        self.goals = goals
        self.p = p


class Animation:
    '''
    An animation is constructed by an array of goal positions (:class:`Keyframe`).
    Between two keyframes, the goal positions are interpolated by an  :class:`Interpolator`.
    '''

    def __init__(self, name, keyframes, default_interpolator=None):
        self.name = name
        self.keyframes = keyframes

def parse(info):
    ''' Diese Methode parst eine Animation aus
        einer :class:`dict` Instanz *info*, wie sie mit
        :func:`as_dict` erzeugt wurde.
    '''
    anim = Animation(info["name"], ())

    keyframes = info.get("keyframes", ())
    anim.keyframes = [Keyframe(k.get('goals', {}), k.get('duration', 1), k.get('pause', 0), k.get('p', {})) for k in
                      keyframes]

    return anim


def as_dict(anim):
    ''' Wandelt die Animation in Standard Python Typen um, damit sie in einem
        Datenformat wie ``.json`` serialisierbar ist.
    '''
    return {
        "name": anim.name,
        "default_interpolator": anim.default_interpolator.__name__,
        "interpolators": {n: ip.__name__ for n, ip in anim.interpolators},
        "keyframes": [{
                          "duration": k.duration,
                          "pause": k.pause,
                          "goals": k.goals,
                          "p": k.p
                      } for k in anim.keyframes]
    }

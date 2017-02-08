#!/usr/bin/env python
#-*- coding:utf-8 -*-
"""
Util
----

Das Util Modul stellt viele nützliche Utils bereit. Für die meisten
stellt es auf Modulebene die Namen zum direktem Import. Dabei werden
von den einzelnen Modulen meist Globale Instanzen gehalten und nur
einzelne Funktionen exportiert, um das I/O möglichst gering zu halten

mittels :func:`get_config` kann die Config geholt werden::

    from bitbots.util import get_config
    config=get_config()
    print config['arms']

.. automodule:: bitbots.util.resource_manager
.. automodule:: bitbots.util.speaker
.. automodule:: bitbots.util.nice
.. automodule:: bitbots.util.math_utils

"""
import rospy
#from bitbots_common.util.resource_manager import find_resource
#from bitbots_common.util.resource_manager import find_animation  # NOQA
#from bitbots_common.util.resource_manager import find  # NOQA
#from bitbots_common.util.resource_manager import generate_find  # NOQA


import os
import errno

try:
    from bitbots_common.utilCython.joints import load_joints_from_yaml
    from bitbots_common.utilCython.joints import JointGroup
    __has_joints = True  # pylint: disable=C0103
except ImportError as e:
    # für entwickler welche sachen ohne virtualenv (meist windows) testen
    # it also happens on crosscompiling
    __has_joints = False  # pylint: disable=C0103

    #from bitbots_common.debug import Scope
    #Scope("Util").warning("bitbots_common.utilCython.joints NICHT gefunden" + str(e))


ERRORS = [
    "UnspecifiedError",
    "InstructionError",
    "OverloadError",
    "CheckSumError",
    "RangeError",
    "OverHeatingError",
    "AngleLimitError",
    "InputVoltageError"

]


def pid_exists(pid):
    """Check whether pid exists in the current process table."""
    if pid < 0:
        return False
    try:
        os.kill(pid, 0)
    except OSError as error:
        return error.errno == errno.EPERM
    else:
        return True

def own_pid(pid):
    return pid == os.getpid()

def get_error_list(errorbits):
    """
    Returns a list of human-readable ERRORs for an mask of errors from the
    motors
    """
    out = []
    for error, msg in zip([bool(errorbits & (1 << 8 - i - 1))
                          for i in range(8)], ERRORS):
        if error:
            out.append(msg)
    return out


class Joints(object):
    """
    Diese Klasse lädt alle vorhandenen Joints und Jointgruppen
    aus einer yaml-datei, und stellt sie dann über verschiedene Methoden
    bereit
    """
    def __init__(self):
        self.joints = rospy.get_param("/joints")

    def all(self):
        """
        Gibt alle bekannten Joints als JointGroup zurück
        """
        return self.joints

    def all_names(self):
        """
        Gitb die Namen aller bekannten Joints zurück
        """
        self.joints.get_names()

    def min(self):
        """
        Gibt alle Joints zurück welche in der Minimalkonfiguration
        vorhanden sind
        """
        return JointGroup([joint for joint in self.joints if joint.cid <= 20])

    def min_mames(self):
        """
        Gibt doe Namen der Joints in der minimalkonfiguration zurück
        """
        return self.min().get_names()

    def actual(self):
        """
        Gibt die Joints zurück die der Roboter nach aktueller Config
        haben sollte
        """
        if rospy.get_param('/hands', False):
            return self.all()
        else:
            return self.min()

    def actual_names(self):
        """
        Gibt die namen der Motoren welche aktuell nach Config vorhanden
        sind zurück
        """
        return self.actual().get_names()

    def actual_names_ordered(self):
        """
        Gibt die Namen der Motoren welche nach aktueller Config
        vorhanden sind zurück, die Motoren sind dabei nach ihrer CID
        sortiert
        """
        motors = dict((joint.cid, joint.name) for joint in self.actual())
        return [motors[cid] for cid in motors]

    def names_ordered(self):
        """
        Gibt die Namen der Motoren zurück, die Motoren sind dabei nach
        ihrer CID sortiert
        """
        motors = dict((joint.cid, joint.name) for joint in self.all())
        return [motors[cid] for cid in motors]

    def get_motor_id(self, name):
        """
        Gibt die ID zu einem Motornamen zurück
        """
        return self.joints.get(name).cid

    def get_motor_name(self, cid):
        """
        gibt den namen zu einer motorid zurück
        """
        return self.joints.get_joint_by_cid(cid).name

    def get_joint_by_cid(self, cid):
        """
        Gibt ein Gelenk nach seiner ID zurück.

        :returns: Joint Objekt
        """
        return self.joints.get_joint_by_cid(cid)

    def get_joint_by_name(self, name):
        return self.joints.get(name)

if not __has_joints:
    # Modul gibts nicht, keine Joint implementierung
    class JointsReplacer(object):  # pylint: disable=R0921, R0903
        """
        Helferklase wen bitbots.util.joints nicht verfügbar ist

        momentan ohne Implementirung
        """
        def __init__(self):
            raise NotImplementedError("da bitbots.util.joints nicht gefunden" +
                                      " wurde, kann Joints nicht " +
                                      "zurverfügung gestellt werden")
    # ersetzen der Jointimplementierung
    Joints = JointsReplacer  # NOQA pylint: disable=C0103


def sign(a):
    """ Simple Sign Implementation """
    if a > 0:
        return 1
    elif a < 0:
        return -1
    else:
        return 0

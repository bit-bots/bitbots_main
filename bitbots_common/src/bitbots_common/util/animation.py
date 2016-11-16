#!/usr/bin/env python
#-*- coding:utf-8 -*-
from bitbots_common.debug import Scope
from bitbots_common.util.resource_manager import find_animation
from bitbots_common.util.config import get_config

import json

import threading

CONFIG = get_config()
debug = Scope("Util.Scope")
try:
    import bitbots_motion.motion.animation as animation
    __anmimation_module = True
except ImportError:
    # für entwickler welche sachen ohne virtualenv (meist windows) testen
    __anmimation_module = False
    debug.warning(
        "Animation Framework NICHT gefunden (bitbots.motion.animation)")
try:
    from bitbots_ipc.ipc import STATE_ANIMATION_RUNNING
except ImportError:
    # für entwickler welche sachen ohne virtualenv (meist windows) testen
    STATE_ANIMATION_RUNNING = 1
    debug.warning("IPC Implementation nicht gefunden")


def __play_animation(name, ipc, callback=None):
    """
    Spielt eine Animation ab
    Wenn IPC gesperrt ist wird false zurückgegeben und die Animation
    nicht Ausgeführt, sonst True.
    Animation läuft dann im hintergrund
    """
    old_callback = None
    if isinstance(name, (tuple, list)) and len(name) == 2:
        name, old_callback = name
        debug.warning("Use of deprecated callback in play_animation")

    if not ipc.controlable:
        debug.log("Verweigere Animation für '%s', da ipc not controllable meldet" % name)
        return False
    elif ipc.get_state() == STATE_ANIMATION_RUNNING:
        debug.log("Verweigere Animation für '%s', da bereits eine animation läuft" % name)
        return False

    # Animation laden
    filename = find_animation(name)
    with open(filename) as fp:
        info = json.load(fp)

    if "hands" in info:
        if info["hands"] == "yes" and not CONFIG["hands"]:
            debug.warning("Versuche Animatione %s abzuspielen: Diese Animation \
                benötigt Hände" % name)
            return False
        elif info["hands"] == "no" and CONFIG["hands"]:
            debug.warning("Versuche Animatione %s abzuspielen: Diese Animation \
                funktioniert nicht mit Händen" % name)
            return False

    anim = animation.parse(info)

    def play(callback):
        if not ipc.controlable or ipc.get_state() == STATE_ANIMATION_RUNNING:
            debug.log("Führe Animations-Callback aus")
            callback(False)
            return

        with ipc.force:
            try:
                debug.log("Spiele Animation %s..." % name)
                animation.Animator(anim, ipc.get_pose()).play(ipc)
                debug.log("Animation %s fertig." % name)
                if callback:
                    debug.log("Führe Animations-Callback aus")
                    callback(True)
            except:
                debug.log("Animation %s wurde durch einen Fehler unterprochen"
                          % name)
                if callback:
                    debug.log("Führe Animations-Callback aus")
                    callback(False)

    debug.log("Starte Animationsthread für %s" % name)
    thread = threading.Thread(target=play, args=(callback,))
    thread.daemon = True
    thread.start()

    if old_callback:
        debug.log("Führe Animations-Callback aus")
        old_callback()

    return True

if __anmimation_module:
    play_animation = __play_animation
else:
    # kompatibilität zum testen under Windows und so
    def __play_animation_dumy(name, ipc, callback=None):
        debug.warning(
            "Animation wird nicht abgespielt da bitbots.motion.animation fehlt")
        return False

    play_animation = __play_animation_dumy

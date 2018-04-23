# -*- coding:utf-8 -*-

import time

import math
import rospy
from bitbots_common.pose.pypose import PyPose as Pose

from bitbots_common.pose.pypose import PyJoint as Joint

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
        self.interpolators = {}
        self.keyframes = keyframes
        self.default_interpolator = default_interpolator or LinearInterpolator

    def get_interpolator(self, name):
        """
            Returns the interpolator class which uses the joint *name*.
            If none is set, the default interpolator is returned
        """
        return self.interpolators.get(name, self.default_interpolator)

    def get_steps(self, nameb: bytes):
        """ Returns list of :class:`Step` objects for the joint *name*
        """
        name = nameb.decode()
        time = 0
        steps = []
        for keyframe in self.keyframes:
            # zeit immer berechnen sonst gibts bei in der mitte fehlenden
            # motoren komische bugs
            time += keyframe.duration
            if keyframe.duration < 0.001:
                rospy.logwarn_throttle(1, "Keyframe has very low duration, this may lead to errors.")

            if name not in keyframe.goals:
                # wenn das gelenk in diesem Keyframe nicht angesteuert wird
                # ignorieren wir es. Das bedeutet auch das die P-Gains dann
                # nicht gesetzt werden können (das würde problkeme mit einem
                # lehren step element geben)

                # wir müssen noch die pause hinzurechnen, sonst ist unsere
                # zwischenzeit eventuell etwas klein
                time += keyframe.pause
                continue

            # Keyframe anfahren

            steps.append(Step(
                time,
                keyframe.goals[name],
                keyframe.p.get(name, -1)
            ))

            if keyframe.pause > 0:
                # Pause beachten
                time += keyframe.pause
                steps.append(Step(time, keyframe.goals[name]))

        return steps


class Step:
    '''
    Simple class for saving splines
    '''

    def __init__(self, time, value, p=-1):
        self.hold = False
        self.off = False
        self.time = time
        self.p = p

        if isinstance(value, str):
            if value == "off":
                self.off = True
            elif value == "hold":
                rospy.logwarn("'hold' noch nicht unterstützt!")
                self.hold = True
            else:
                raise ValueError("Ungültiger Wert fürs Ziel:" + str(value))

            # gegen einen anderen Wert ersetzten
            value = 0

        self.value = value

    def __repr__(self):
        return "<Step time=%1.2f, value=%1.2f, p=%d>" % (self.time, self.value, self.p)


class Interpolator:
    """
    This is the basic class for all interpolators.
    Interpolated values can be get by calling the  :func:`interpolate`
    """

    def __init__(self, steps):
        self.steps = steps
        if not self.steps:
            raise ValueError("'steps' cannot be empty")

        # Den Wert für deaktivierte Stützstellen auf den Wert der vorigen
        # Stützstelle setzen
        for idx in range(1, len(self.steps)):
            next = self.steps[idx]
            prev = self.steps[idx - 1]
            if next.hold or next.off:
                next.value = prev.value

        # kleinster und größster Zeitpunkt der Stützstellen
        self.time_min = (self.steps[0]).time
        self.time_max = (self.steps[-1]).time

        self.prepare()

    def prepare(self):
        ''' Diese Methode kann überladen werden, um die im Konstruktor
            übergebenen Daten für die
            :func:`interpolate`-Methode vorzubereiten.
        '''
        pass

    def interpolate(self, t):
        ''' Interpolates a value at time t
            returns a tuple with 4 elemtens:
            # goal angle
            # Bool, if joint is activ
            # Bool, if joint schould stay at current position
            # Integer, p value, -1=dont change
        '''
        return 0, False, False, -1


class LinearInterpolator(Interpolator):
    '''
    Simple linear interpolation
    '''

    def interpolate(self, t):
        last = None
        for next in self.steps:
            if last is None:
                # Wir sind vor dem Anfang
                if t <= next.time:
                    return (next.value, next.off, next.hold, next.p)

            elif last.time < t <= next.time:
                # in diesem Interval
                dt = (next.time - last.time)
                dv = (next.value - last.value)
                return (
                    last.value + ((t - last.time) / dt) * dv,
                    next.off, next.hold, last.p  # TODO: überprüfen ob das nicht alles last sein möchte
                )

            # letze Position merken
            last = next

        # letzten Wert zurückgeben
        return (last.value, last.off, last.hold, -1)


def cubic_hermite_interpolate(a, b, t):
    dt = b.time - a.time
    t = (t - a.time) / dt
    return (2 * t * t * t - 3 * t * t + 1) * a.value \
           + (t * t * t - 2 * t * t + t) * dt * a.m \
           + (t * t * (3 - 2 * t)) * b.value \
           + (t * t * (t - 1)) * dt * b.m


class CubicHermiteInterpolator(Interpolator):
    """
    `Cubic-Hermite <http://en.wikipedia.org/wiki/Cubic_Hermite_spline>`_-Splines.
    """

    def interpolate(self, t):
        last = None
        for next in self.steps:
            if last is None:
                # Wir sind vor dem Anfang
                if t <= next.time:
                    return (next.value, next.off, next.hold, next.p)

            elif last.time < t <= next.time:
                # wir sind im richtigen Interval
                return (
                    cubic_hermite_interpolate(last, next, t),
                    next.off, next.hold, last.p  # TODO: überprüfen ob das nicht alles last sein müsste
                )

            # letze Position merken
            last = next

        # letzten Wert zurückgeben
        return (last.value, last.off, last.hold, -1)


class CatmullRomInterpolator(CubicHermiteInterpolator):
    """
    `Catmull-Rom <http://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull.E2.80.93Rom_spline>`
    """

    def m(self, idx):
        if idx == 0 or idx == len(self.steps) - 1:
            return 0

        a = self.steps[idx - 1]
        b = self.steps[idx + 1]
        return (b.value - a.value) / float(b.time - a.time)

    def prepare(self):
        for idx in range(len(self.steps)):
            (self.steps[idx]).m = self.m(idx)


class Animator:
    """
    The :class:`Animator` plays animations.
    The constructor takes a finished animation object for which the interpolator instance is created

    Wird dem Konstruktor eine weitere Pose in *first_pose* übergeben, so
    wird dies als die aktuelle *ist Position* interpretiert und als Ausgang
    für die Animation genutzt. Ist der Parameter ``None``, so geht der
    :class:`Animator` davon aus, dass sich der Roboter bereits in der ersten
    Pose der :class:`Animation` befindet.
    """

    def __init__(self, anim, first_pose=None):
        self.name = anim.name
        self.interpolators = {}
        for joint in Pose().names:
            ip = anim.get_interpolator(joint)
            steps = anim.get_steps(joint)
            if not steps:
                continue

            if first_pose is not None:
                steps.insert(0, Step(0, first_pose[joint].position))

            self.interpolators[joint] = ip(steps)

        # Start und Endzeit
        values = list(self.interpolators.values())
        self.time_min = min(ip.time_min for ip in values)
        self.time_max = max(ip.time_max for ip in values)
        self.duration = self.time_max - self.time_min
        self.t_start = None
        self.speed_factor = rospy.get_param("animation/speed_factor")

    def get_pose(self, t, pose=None):
        ''' Interpolates a pose at time t
        '''

        if pose is None:
            pose = Pose()

        for name, joint in pose.joints:
            if name in self.interpolators:
                # Werte interpolieren
                ip = self.interpolators[name]
                goal, off, hold, p = ip.interpolate(t)

                if off:
                    # Gelenk locker stellen
                    joint.set_active(False)
                else:
                    if not hold:
                        # Ziel für das Gelenk setzen
                        joint.set_goal(goal)

                    # TODO hold geht noch nicht, weil wir den Motor nicht
                    # aktivieren können, ohne seine Position zu verändern.

                    if p != -1:
                        joint.set_p(p)
        return pose

    def playfunc(self, stepsize):
        """ This function generates an anymous function which returns at each caLL a new pose
            The returned function has to be called at least each *stepsize* seconds
            but can be called more often
            None is returned when the animation is finished
        """
        pre = stepsize * 1.5

        self.t_start = rospy.get_time()

        def update(current):
            t_robo = (rospy.get_time() - self.t_start) + self.time_min

            if t_robo > self.time_max:
                return None

            t_pose = min(t_robo + pre, self.time_max)
            next = self.get_pose(t_pose)
            for name, joint in next.joints:
                if joint.has_changed():
                    # Berechne Geschwindigkeit für dieses Gelenk
                    curjoint = current[name]
                    if t_pose - t_robo != 0:
                        degree_per_second = abs(joint.get_goal() - curjoint.get_position()) / (t_pose - t_robo)
                    else:
                        degree_per_second = abs(joint.get_goal() - curjoint.get_position()) / 0.01
                    speed = degree_per_second * pre * self.speed_factor  # see config for docu
                    joint.set_speed(speed)
            return next

        return update

    def get_duration(self):
        return self.duration

    def get_start_time(self):
        return self.t_start

    def __str__(self):
        return "<Animator '%s' duration=%1.2fsek>" % (self.name, self.duration)


INTERPOLATORS = {
    "LinearInterpolator": LinearInterpolator,
    "CatmullRomInterpolator": CatmullRomInterpolator
}


def parse(info):
    ''' Diese Methode parst eine Animation aus
        einer :class:`dict` Instanz *info*, wie sie mit
        :func:`as_dict` erzeugt wurde.
    '''
    anim = Animation(info["name"], ())

    if "default_interpolator" in info:
        anim.default_interpolator = INTERPOLATORS[info["default_interpolator"]]

    interpolators = info.get("interpolators", {})
    anim.interpolators = {name: INTERPOLATORS[ip] for name, ip in interpolators.items()}

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

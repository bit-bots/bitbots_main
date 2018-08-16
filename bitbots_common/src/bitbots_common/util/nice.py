#-*- coding:utf-8 -*-
"""
Nice
^^^^

Dieses Modul vereinfacht die Manipulation des Nicelevels
"""
import os
import time
import multiprocessing
import rospy

from bitbots_common.util import get_config


class Nice(object):
    """
    Diese Klasse stellt methoden bereit, um die relevanz des aktuellen
    Prozesses zu verändern.

    Wenn das runtersetzen des Nicelevels (hochsetzen der Priorität) nicht
    möglich ist wird aus gründen der sicherheit nichts getan.
    """
    def __init__(self, debug=None):
        config = get_config()
        if config['nice']:
            try:
                self.nice_ok = True
                self.level = self.get_nice()
                self.__test_nice()
            except:  # behandlung um intern consistent zu bleiben
                self.nice_ok = False
                raise  # fehler weiterwerfen
        else:
            rospy.loginfo("Nice ist in der Config deaktiviert")
            self.nice_ok = False

    def __test_nice(self):
        """
        Testet ob das runtersetzen des nicelevels möglich ist
        """
        try:
            # wenn wir hier unterbrochen werden passieren dumme dinge
            try:
                self.set_nice(-1, True)
                self.set_nice(1, True)
                self.nice_ok = 1
            except OSError:
                # dann der lange weg
                try:
                    os.popen("sudo -n renice -n %d -p %d 2> /dev/null" %
                        (-1, multiprocessing.current_process().pid))
                except OSError:
                    #irgentwass schiefgegangen, der rest fängt es ab...
                    pass
                time.sleep(1)
                if not (self.get_nice() == -1):
                    # wir dürfen nicht.... :(
                    self.nice_ok = False
                    rospy.logwarn("Nicelevel darf nicht reduziert werden, disable Nice")
                else:
                    self.nice_ok = 2
                    self.set_normal(True)
        except:
            # dann gehts nicht
            # wenn wir hier landen wird es hoffentlich meist ein
            # KeybordInterrupt sein, wenn wir es nicht auf false setzen
            # passieren unter umständen komische dinge
            self.nice_ok = False
            raise  # weitergeben des fehlers...

    def get_active(self):
        """
        :return: Ob das Modul aktiv ist.
        :return type: boolean
        """
        return self.nice_ok

    def get_nice(self):
        """
        :return: Das aktuelle Nicelevel
        :return type: int
        """
        self.level = os.nice(0)
        rospy.loginfo("niceines", self.level)
        return self.level

    def set_nice(self, change, silence=False):
        """
        Verändert das Nicellevel um change

        :param change: die Änderung des Nicelevels
        :type change: int
        :param silence: Supress debugmessages
        :type silence: boolean
        :return: True wenn erfolgt
        :return type: boolean

        .. hint:: Wenn :func:`get_nice` == False wird ''nichts'' getan
            (außer einer debug warning)

        """
        if self.nice_ok:
            if self.nice_ok == 1:
                self.level = os.nice(change)
            else:
                os.popen("sudo -n renice -n %d -p %d 2> /dev/null" %
                    (self.level + change,
                        multiprocessing.current_process().pid))
                time.sleep(1)
                self.get_nice()
            if not silence:
                rospy.loginfo("Set Nicelevel to %d" % self.level)
            return True
        else:
            rospy.logwarn("Setzen von Nice nicht möglich")
            return False

    def set_realtime(self):
        """
        Setzt die Priorität auf "Realtime"
        """
        if self.nice_ok:
            return self.set_level(-20)
        else:
            rospy.logwarn("Set (Soft-) Realtime Priorität nicht möglich!")
            return False

    def set_normal(self, silence=False):
        """
        Setzt die Prioritöt auf Normal
        """
        return self.set_level(0, silence)

    def set_level(self, level, silence=False):
        """
        Setzt das nice level auf level
        :param level: das Level auf das die Priorität gesetzt werden soll
        :type level: int
        """
        return self.set_nice((self.level - level) * (-1), silence)

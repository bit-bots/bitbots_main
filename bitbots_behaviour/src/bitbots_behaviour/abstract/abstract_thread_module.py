# -*- coding:utf-8 -*-
"""
AbstractThreadModule
^^^^^^^^^^^^^^^^^^^^

Diese Modul stellt die Infratsruktur bereit um ein Modul zu schaffen
welches seine arbeit in einem eigenen Thread erledigt. Dieses Modul
sorgt dabei für die Kommunikation mit dem maninthread. Es ist dazu
gedacht um von anderen Modulen beerbt zu werden, nicht um selbst
ausgeführt zu werden.

.. warning::
    Ein Thread in Python bedeutet nicht das es völlig unabhängig vom
    Rest auf einem eigenen CPU-Kern laufen kann, in Python kann immer
    nur ein Thread zu zeit arbeiten. Das bedeutet das sich Threads
    überwiegend für Module eigenen die auf I/O zu warten haben

.. seealso: `MultiProcessModule` lässt Module echt unabhängig laufen
"""

import threading

from bitbots.debug import Scope
from bitbots.modules.abstract import AbstractModule


class AbstractThreadModule(AbstractModule):
    """
    In diesem Modul wird :func:`run` in einem eigenen Thread ausgeführt
    und bietet die Möglichkeit durch Vererbung leicht Module in einem
    eigenen Thread arbeiten zu lassen. Dieses Modul kümmert sich dabei
    um die Kommunikation des data dictonarrys.

    .. warning::
        innerhalb von :func:`run` darf und kann nicht auf data
        zugegriffen werden, dafür gibt es die Funktionen :func:`get`
        und :func:`set`. Außerdem sollte die :func:`update` niemals
        überschrieben werden, da sonst die Kommunikation mit den
        restlichen Modulen nichtmehr funktioniert.
    """

    def __init__(self, requires=(), provides=()):
        """
        :param requires: alle Datenfelder welche innerhalb von
            :func:`run` benötigt werden
        :type requires: list of strings
        :param provides: alle Datenfelder welche von diesem Modul
            nach außen gesetzt werden
        :type requires: list of strings

        .. warning:: es werden nur datenfelder von und nach dem normalen
            datadictonarry kommuniziert welche auch in den requieres
            bez. provides stehen.
        """
        self.lock = threading.RLock()

        # Dictionary für den Datenaustausch
        self.__internal = {}
        self.requires = requires
        self.provides = provides
        self.__debug = Scope("Threading.%s" % self.__class__.__name__)

        self.started = False

    def start(self, data):
        """
        Initiallisiert alle Provides im globalem data dictonarry.
        wenn noch kein wert mittels :func:`set` in init gesetzt wurde
        werden die datenfelder auf None initiallisiert. Dies dient dazu
        das es nicht vorkommt das providete Felder im dict nicht
        existieren und damit zu fehlern beim zugriff in anderen Modulen
        führen.
        """
        for name in self.provides:
            data[name] = self.__internal.get(name, None)

    def __start(self):
        """
        Interner Starthelfer, sorgt dafür das Fehler beim Debug landen
        """
        self.__debug("Thread gestartet")
        try:
            self.run()
        except Exception as e:
            self.__debug.error(e, "%s has exitet with an Exeption" % str(self))
            self.__debug("Versuche einen Neustart")
            self.started = False
        else:
            self.__debug("Thread wurde beendet")

    def run(self):
        """ Diese Funktion wird in einem Thread ausgeführt """
        raise NotImplementedError

    def get(self, name, default=None):
        """
        Diese Methode holt einen Wert aus dem globalem data dictonarry.
        Durch das Threading kann es dabei aber zu leichten verzögerungen
        kommen.

        :param name: Der name des zu hohlenden Datenfeldes
        :type name: String
        :param default: Wert der Zurückgegeben wird, wenn name noch
            nicht gesetzt ist
        """
        with self.lock:
            return self.__internal.get(name, default)

    def set(self, name, value):
        """
        Diese Methode setzt einen Wert unter name im data dictonarry.
        """
        with self.lock:
            self.__internal[name] = value

    def update(self, data):
        """
        Hier wird sich darum gekümmert das daten die mittels :func:`set`
        gesetzt wurden im data dictonarry ankommen und die daten aus
        require vom dictonarry ausgelesen und zwischengespeichert, so
        dass sie mittels :func:`get` geholt werden können.

        Beim ersten durchlauf wird hier auch der Thread gestartet.
        """
        with self.lock:
            for name in self.requires:
                if name in data:
                    self.__internal[name] = data[name]

            if not self.started:
                # Wenn der Therad noch nicht läuft, starten
                self.started = True
                thread = threading.Thread(target=self.__start)
                thread.daemon = True
                thread.start()

            for name in self.provides:
                data[name] = self.__internal.get(name, None)

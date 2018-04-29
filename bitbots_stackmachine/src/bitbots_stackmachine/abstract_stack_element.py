# -*- coding:utf-8 -*-
"""
AbstractStackElement
^^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: Nils Rokita <0rokita@informatik.uni-hamburg.de>

"""


class AbstractStackElement(object):
    """
    Das AbstractStackElement ist die Grundlager aller Module welche auf dem
    Stack landen. Es hat dafür einige hilfsfunktionen welche nicht
    überladen werden sollten. Die eigendliche Arbeit wird in :func:`perform`
    ausgeführt. Jedes Modul welches von AbstractStackElement erbt kann
    als Startmodul eines Verhaltens gewählt werden.
    """
    _behaviour = None
    reevaluate = False
    _init_data = None

    def __init__(self, connector, args=None):
        pass

    def setup_internals(self, behaviour, init_data):
        """
        Diese Methode initialisiert die internen Variablen, und wird von der
        Stackmachine aufgeruffen

        :param behaviour: Die VerhaltensStackmachine die das Modul auf
            dem Stack hat.
        :param init_data: Die Daten die bei __init__ übergeben wurden,
            ist für das reevaluate
        """
        self._behaviour = behaviour
        self._init_data = init_data

    def get_init_data(self):
        """
        Hohlt die init daten des  Modules um nachzusehen ob die jetzt anders
        währen.
        """
        return self._init_data

    def pop(self):
        """
        Hilfsmethode zum einfachen pop vom Stack.

        Diese Methode sollte immer mit return aufgeruffen werden::
            return self.pop()

        """
        self._behaviour.pop()

    def push(self, module, init_data=None):
        """
        Hilfsmethode zum einfachen push auf den Stack.

        Sollte immer mit return aufgeruffen werden::
            return self.push(NeuesModul, data)


        :param module: Das auf den Stack zu legende Modul
        :type module: Class, Vererbt von AbstractStacklement
        :param init_data: Daten die dem Modul beim init übergeben werden
        :type init_data: object
        """
        self._behaviour.push(module, init_data)

    def perform(self, connector, reevaluate=False):
        """
         Diese Methode wird aufgeruffen wenn das Modul im Stack ganz oben
         liegt und drann ist. Diese Methode sollte überladen werden!

         :param reevaluate: Ob der aktuelle Aufruff ein reevaluate des
            zustandes ist
        """
        msg = "You schuld overrride perform() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def interrupt(self):
        """
        Ein Interrupt fürt zu einem neuaufbauen des Stacks
        """
        self._behaviour.interrupt()

    def get_reevaluate(self):
        """
        Gibt zurück ob dieses Modul reevaluiert werden muss.
        """
        return self.reevaluate

    def do_not_reevaluate(self):
        """
        Unterdrückt das nächste reevaluate
        """
        self._behaviour.set_do_not_reevaluate()

    def __repr__(self):
        """
        Wir kürzen die Repräsentation ab, ist so kürzer, und sagt
        trotzdem noch genug
        """
        return "<AbstractStrackElement: " + \
               self.__class__.__module__.split('.')[-2] \
               + "." + self.__class__.__name__ + ">"

    @staticmethod
    def sign(x):
        return -1 if x < 0 else 1

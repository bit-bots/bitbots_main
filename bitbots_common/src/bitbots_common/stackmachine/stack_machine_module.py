# -*- coding:utf-8 -*-
"""
StackMachineModule
^^^^^^^^^^^^^^^^^^

.. moduleauthor:: Nils Rokita <0rokita@informatik.uni-hamburg.de>

History:

* 1.12.13: Created (Nils Rokita)
* 14.12.13: Umstellung auf Weltmodellschnittstelle (Martin Poppinga)
* 05.02.13: Erweterung um reevaluate von decisions (Nils Rokita)
* 03.04.14: Unterdrückung des reevaluate (Nils Rokita)
"""
import time
import rospy
from modell.connector import Connector


class StackMachineModule:
    """
    Diese Klasse handeld die Verhaltensarchitektur
    """

    stack = []
    start_module = None
    start_module_data = None
    stack_excec_index = -1
    stack_reevaluate = False
    do_not_reevaluate = False
    old_representation = ""

    def __init__(self):
        self.connector = Connector()

    def _init_module(self, module, init_data=None):
        """
        Initialisiert das Modul module
        """
        mod = module(init_data)
        mod.setup_internals(self, init_data)
        return mod

    def set_start_module(self, start_module, init_data=None):
        """
        Mit Dieser Methode wird das Start DecisionModule festgelegt,
        welches immer als unterstes auf dem Stack verbleibt.

        Diese Methode sollte meist im __init__ aufgeruffen werden
        """
        self.stack = []
        self.start_module = start_module
        self.start_module_data = init_data
        self.stack.append(self._init_module(start_module, init_data))

    def interrupt(self):
        """
        es ist etwas passiert so das das verhalten sich komplett neu
        entscheiden muss, und den stack daher neu aufbaut.
        """
        if self.stack_reevaluate:
            # wir waren gerade dabei vorbedingungen zu prüfen
            # wir höhren natürlich damit auf...
            self.stack_reevaluate = False
            # damit merkt update() das es aufhöhren muss
        self.stack = [self._init_module(self.start_module,
                                        self.start_module_data)]

    def event_interrupt(self):
        """
        Diese Methode wird aufgerufen wenn ein GLOBAL_INTERRUPT Event kommt.

        Tut in der Standartimplementation nichts außer es durchreichen.
        """
        self.interrupt()

    def update(self, reevaluate=True):
        """
        Führt das Modul was oben auf dem Stack liegt aus. Prüft vorher
        etwagige vorbedingungen (Module die Reevaluate gesetzt haben)

        :param: reevaluate: Ob der Stack reevaluiert werden soll
        :type reevaluate: bool
        """
        if reevaluate and not self.do_not_reevaluate:
            self.stack_excec_index = 0
            self.stack_reevaluate = True
            for element in self.stack[:-1]:
                # alle elemente außer das letzte überprüfen ob wir
                # die entscheidung reevaluieren müssen
                if element.get_reevaluate():
                    element.perform(self.connector, True)
                    if not self.stack_reevaluate:
                        # Beim reeavluieren ist irgendwo abgebrochen
                        # worden, wir höhren hier auf
                        return
                self.stack_excec_index += 1
            self.stack_reevaluate = False
        # Die eigendliche ausführung des Moduls
        if reevaluate:
            # flag zurücksetzten dass wir nicht reevaluieren
            self.do_not_reevaluate = False
        self.stack[-1].perform(self.connector)

    def post(self):

        if self.stack != self.old_representation:
            self.old_representation = self.stack
            a = [str(e).split(" ")[1][0:-1] for e in self.stack]
            b = [str(c).split(".")[-1] for c in a]

            rospy.logdebug("Stack1" + " ".join(b))

    def push(self, module, init_data=None):
        """
        Ein neues Modul auf den Stack legen, es wird sofort ausgeführt

        .. warning::
            Nach push sollte niemals noch etwas ausgefphrt werden, es kann
            sonst zu massiven verwirrungen kommen. Am besten immer::

                return self.push(xxxDecisionModule, data)

            nutzen

        :param module: Das auf den Stack zu legende Modul
            (Nicht inizialisierne!)
        :type module: Module
        :param init_data: init Data wird dem neuen Modul zum Init
            übergeben, optional
        """
        if self.stack_reevaluate:
            # wir sind gerade dabei die vorbedingungen zu prüfen
            # testen op die entscheidung noch die gleiche ist:
            if type(self.stack[self.stack_excec_index + 1]) == module and \
                            self.stack[self.stack_excec_index + 1].get_init_data() \
                            == init_data:
                # entscheidung ist gleich wir tun nichts
                return
            else:
                # andere entscheidung, wir bauen den Stack bis hier
                # ab und puschen das neue rauf.
                self.stack = self.stack[0:self.stack_excec_index + 1]
                # reevaluate ist hiermit zuende
                self.stack_reevaluate = False
        self.stack.append(self._init_module(module, init_data))
        # wir rufen das neue Modul ohne reevaluation auf
        self.update(False)

    def pop(self):
        """
        Entfern sich selber vom Stack, der vorgänger wird in diesem
        Frame _nicht_ mehr aufgeruffen.
        """
        if len(self.stack) > 1:
            if self.stack_reevaluate:
                # wir reevaluieren gerade, wir kürzen den stack hier ein
                # inklusive das aktuelle Modul
                if self.stack_excec_index > 0:
                    # nur stack kürzen wenn er dann noch ein element hatt
                    self.stack = self.stack[0:self.stack_excec_index]
                # mit dem reevaluieren aufhöhren
                self.stack_reevaluate = False
            else:
                self.stack.pop()
        else:
            rospy.logwarn("Can't pop in %s: stack is empty, Module: %s" % (repr(self), repr(self.stack[0])))

    def set_do_not_reevaluate(self):
        """Beim nächsten update wird nicht reevaluiert"""
        self.do_not_reevaluate = True

    def get_stack(self):
        """
        Gibt den Aktuellen Stack zurück
        """
        return self.stack

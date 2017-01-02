# -*- coding:utf-8 -*-
"""
testAbstractStackElement
^^^^^^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: Nils Rokita <0rokita@informatik.uni-hamburg.de>

History:

* 4.12.13: Created (Nils Rokita)


TEsts für ein STackElement, lässt sich erben um zu garantieren das bei
implmentationen diese Abstrakten elements alle wichtigen funktionen
ordcnungsgemäß arbeiten

"""
import unittest
from bitbots.modules.abstract.stack_machine_module import StackMachineModule
from bitbots.modules.abstract.abstract_stack_element import AbstractStackElement
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.abstract.abstract_action_module import AbstractActionModule
from bitbots.debug import Scope


class TestStackElement1(AbstractStackElement):
    def __init__(self, obj):
        self.count = 0

    def perform(self, data):
        self.count += 1


class BehaviourStub(StackMachineModule):
    debug = Scope("Test.BehaviourStub")

    def __init__(self):
        StackMachineModule.__init__(self)
        StackMachineModule.set_start_module(self, TestStackElement1)
        self.pops = 0
        self.pushs = 0
        self.interrupts = 0
        self.connector = None
        self.push_data = None

    def interrupt(self):
        StackMachineModule.interrupt(self)
        self.interrupts += 1

    def pop(self):
        StackMachineModule.pop(self)
        self.pops += 1

    def push(self, module, init_data=None):
        StackMachineModule.push(self, module, init_data)
        self.pushs += 1
        self.push_data = (module, init_data)

    def set_connector(self, connector):
        self.connector = connector


class BehaviourStub2(BehaviourStub):
    """ Wird von anderen zum testen benutzt"""

    def interrupt(self):
        self.interrupts += 1

    def pop(self):
        self.pops += 1

    def push(self, module, init_data=None):
        self.pushs += 1
        self.push_data = (module, init_data)


class TestAbstractStackElement(unittest.TestCase):
    stackElement = AbstractStackElement

    @classmethod
    def setUpClass(cls):
        print("#### Test AbstractStackElement ####")

    def setupAbstractStackElement(self):
        behaviour = BehaviourStub()
        cls = self.stackElement()
        cls.setup_internals(behaviour, None)
        return cls, behaviour

    def testSetupInternals(self):
        behaviour = BehaviourStub()
        cls = self.stackElement()
        init_data = {1: 'huio'}
        cls.setup_internals(behaviour, init_data)
        self.assertEquals(behaviour, cls._behaviour)
        self.assertEquals(init_data, cls._init_data)
        self.assertEquals(init_data, cls.get_init_data())

    def testPush(self):
        cls, behaviour = self.setupAbstractStackElement()
        cls.push(TestStackElement1)
        self.assertEquals(TestStackElement1, behaviour.push_data[0])
        self.assertEquals(None, behaviour.push_data[1])

    def testPush_with_init_data(self):
        cls, behaviour = self.setupAbstractStackElement()
        data = "gzguorehjfio rneguzf reoiufiou ergzifg heöroiuzf34"
        cls.push(TestStackElement1, data)
        self.assertEquals(TestStackElement1, behaviour.push_data[0])
        self.assertEquals(data, behaviour.push_data[1])

    def testPop(self):
        cls, behaviour = self.setupAbstractStackElement()
        cls.pop()
        self.assertEquals(1, behaviour.pops)
        cls.pop()
        self.assertEquals(2, behaviour.pops)

    def testInterrupt(self):
        cls, behaviour = self.setupAbstractStackElement()
        cls.interrupt()
        self.assertEquals(1, behaviour.interrupts)
        cls.interrupt()
        self.assertEquals(2, behaviour.interrupts)

    def testDecide_implemented(self):
        cls, behaviour = self.setupAbstractStackElement()
        # wir machen hier mal ne unterscheidung ob das direkt war oder
        # geerbt wurde
        if self.stackElement in (AbstractStackElement, AbstractActionModule,
                                 AbstractDecisionModule):
            # das abstracte Element
            try:
                cls.perform({})
                self.fail("Da sollte ein NotImplementedError kommen")
            except NotImplementedError as e:
                pass
        else:
            try:
                cls.perform({})
            except NotImplementedError as e:
                raise
            except Exception as e:
                pass
                # ist ok da meist das datadict nicht richtig gefüllt sein wird

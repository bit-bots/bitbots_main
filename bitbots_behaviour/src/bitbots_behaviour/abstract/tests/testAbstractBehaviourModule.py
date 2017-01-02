# -*- coding:utf-8 -*-
import unittest
from bitbots.modules.abstract.stack_machine_module import StackMachineModule
from bitbots.modules.abstract.abstract_stack_element import AbstractStackElement
from bitbots.debug import Scope


class TestStackElement1(AbstractStackElement):
    def __init__(self, obj):
        self.count = 0

    def perform(self, connector, reevaluate=False):
        self.count += 1


class TestStackElement2(TestStackElement1):
    pass


class TestStackElement3(TestStackElement1):
    def __init__(self, obj):
        super(TestStackElement3, self).__init__(obj)
        self.test_in_data = 0

    def perform(self, connector, reevaluate=False):
        super(TestStackElement3, self).perform(connector)
        if connector.is_key_in_data('test'):
            self.test_in_data = 1


class TestStackElement5(TestStackElement3):
    def get_reevaluate(self):
        return True


class TestStackElement4(TestStackElement1):
    def __init__(self, obj):
        super(TestStackElement4, self).__init__(obj)
        self.obj = obj

    def get_init_data(self):
        return self.obj


class EventFrameworkMock(object):
    def send_event(self, event):
        pass

    def register_to_event(self, event, function):
        pass


def create_StackMachineModule_with_start_module_and_data(start_module, init_data=None):
    obj = StackMachineModule()
    obj.internal_init(Scope("Test.Module"), EventFrameworkMock())
    data = {}
    obj.start(data)
    obj.set_start_module(start_module, init_data)
    return obj, data


class TestAbstractBehaviourodule(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        print("#### Test AbstractBehaviour ####")

    def test_internal_init(self):
        obj = StackMachineModule()
        event_framework = EventFrameworkMock()
        obj.internal_init(Scope("Test.Module"), event_framework)
        self.assertEquals(type(obj.debug), Scope)
        self.assertEquals(obj.debug.get_name(), "Test.Module.StackMachine")
        self.assertEquals(len(obj.get_stack()), 0)
        self.assertEquals(event_framework, obj._event_framework)

    def test_set_start_module(self):
        obj, data = create_StackMachineModule_with_start_module_and_data(TestStackElement1)
        self.assertEquals(len(obj.get_stack()), 1)
        self.assertEquals(type(obj.get_stack()[-1]), TestStackElement1)

    def test_set_start_module_init_data(self):
        init_data = "huihui hiufüo#ph p#hf823j "
        obj, data = create_StackMachineModule_with_start_module_and_data(TestStackElement4, init_data)
        self.assertEquals(len(obj.get_stack()), 1)
        self.assertEquals(type(obj.get_stack()[-1]), TestStackElement4)
        self.assertEquals(obj.get_stack()[-1].get_init_data(), init_data)

    def test_update(self):
        obj, data = create_StackMachineModule_with_start_module_and_data(TestStackElement3)
        data["test"] = 1
        obj.update(data)
        self.assertEquals(obj.get_stack()[-1].count, 1)
        self.assertEquals(obj.get_stack()[-1].test_in_data, 1)

    def test_push(self):
        obj, data = create_StackMachineModule_with_start_module_and_data(TestStackElement2)
        data["test"] = 1
        self.assertEquals(len(obj.get_stack()), 1)
        self.assertEquals(type(obj.get_stack()[-1]), TestStackElement2)
        obj.push(TestStackElement3)
        self.assertEquals(len(obj.get_stack()), 2)
        self.assertEquals(type(obj.get_stack()[-1]), TestStackElement3)
        # überprüfen obs gecallt wurde
        self.assertTrue(obj.get_stack()[-1].count > 0)
        self.assertEquals(obj.get_stack()[-1].test_in_data, 1)

    def test_push_reevaluate(self):
        obj, data = create_StackMachineModule_with_start_module_and_data(TestStackElement2)
        data["test"] = 1
        self.assertEquals(len(obj.get_stack()), 1)
        self.assertEquals(type(obj.get_stack()[-1]), TestStackElement2)
        obj.push(TestStackElement2)
        obj.push(TestStackElement2)
        obj.push(TestStackElement2)
        self.assertEquals(len(obj.get_stack()), 4)
        self.assertEquals(type(obj.get_stack()[-1]), TestStackElement2)
        obj.stack_excec_index = 1
        obj.stack_reevaluate = True
        obj.push(TestStackElement3)
        self.assertEquals(len(obj.get_stack()), 3)
        self.assertEquals(type(obj.get_stack()[-1]), TestStackElement3)
        self.assertFalse(obj.stack_reevaluate)
        # überprüfen obs gecallt wurde
        self.assertTrue(obj.get_stack()[-1].count > 0)
        self.assertEquals(obj.get_stack()[-1].test_in_data, 1)

    def test_update_reevaluate(self):
        obj, data = create_StackMachineModule_with_start_module_and_data(TestStackElement3)
        data["test"] = 1
        obj.push(TestStackElement3)
        obj.push(TestStackElement5)
        obj.push(TestStackElement3)
        obj.update(data)
        self.assertEquals(obj.get_stack()[-2].count, 2)
        self.assertEquals(obj.get_stack()[-1].count, 2)
        self.assertEquals(obj.get_stack()[-3].count, 1)
        self.assertEquals(obj.get_stack()[-1].test_in_data, 1)
        self.assertFalse(obj.stack_reevaluate)

    def test_push_init_data(self):
        initdata = "gfpi48z8ü#iu9 ni#uif98"
        obj, data = create_StackMachineModule_with_start_module_and_data(TestStackElement2)
        obj.push(TestStackElement4, initdata)
        self.assertEquals(obj.get_stack()[-1].get_init_data(), initdata)

    def test_update_with_stack(self):
        obj, data = create_StackMachineModule_with_start_module_and_data(TestStackElement3)
        obj.push(TestStackElement3)
        obj.push(TestStackElement3)
        obj.push(TestStackElement3)
        self.assertEquals(obj.get_stack()[-1].test_in_data, 0)
        data['test'] = 1
        obj.update(data)
        self.assertEquals(obj.get_stack()[-1].count, 2)  # 2 weil push auch update aufruft
        self.assertEquals(obj.get_stack()[-1].test_in_data, 1)

    def test_interrupt(self):
        obj, data = create_StackMachineModule_with_start_module_and_data(TestStackElement2)
        obj.push(TestStackElement3)
        obj.push(TestStackElement3)
        obj.push(TestStackElement3)
        self.assertEquals(len(obj.get_stack()), 4)
        obj.interrupt()
        self.assertEquals(len(obj.get_stack()), 1)
        self.assertEquals(type(obj.get_stack()[-1]), TestStackElement2)

    def test_interrupt_with_init_data(self):
        init_data = 'hgui hf8uäp 9izu '
        obj, data = create_StackMachineModule_with_start_module_and_data(TestStackElement4, init_data)
        obj.push(TestStackElement3)
        obj.push(TestStackElement3)
        obj.push(TestStackElement3)
        self.assertEquals(len(obj.get_stack()), 4)
        obj.interrupt()
        self.assertEquals(len(obj.get_stack()), 1)
        self.assertEquals(type(obj.get_stack()[-1]), TestStackElement4)
        self.assertEquals(obj.get_stack()[-1].get_init_data(), init_data)

    def test_pop_with_stack(self):
        obj, data = create_StackMachineModule_with_start_module_and_data(TestStackElement2)
        obj.push(TestStackElement3)
        obj.push(TestStackElement3)
        obj.push(TestStackElement3)
        self.assertEquals(len(obj.get_stack()), 4)
        self.assertEquals(obj.get_stack()[-2].count, 1)
        obj.pop()
        self.assertEquals(len(obj.get_stack()), 3)
        self.assertEquals(obj.get_stack()[-1].count, 1)

    def test_pop_with_emty_stack(self):
        obj, data = create_StackMachineModule_with_start_module_and_data(TestStackElement2)
        self.assertEquals(len(obj.get_stack()), 1)
        obj.pop()
        self.assertEquals(len(obj.get_stack()), 1)
        self.assertEquals(type(obj.get_stack()[-1]), TestStackElement2)

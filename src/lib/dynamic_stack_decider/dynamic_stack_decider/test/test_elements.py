"""
Unit tests for the element base classes (``AbstractStackElement``, ``AbstractActionElement``,
``AbstractDecisionElement``): the helper methods, debug data handling, the reevaluation flags and
the delegation of ``pop`` / ``interrupt`` / ``do_not_reevaluate`` to the DSD.
"""

import pytest

from dynamic_stack_decider.abstract_stack_element import AbstractStackElement

from .runtime_elements import Blackboard, Chooser, Guard, Idle


class FakeDSD:
    """Records the calls the elements delegate to the DSD."""

    def __init__(self):
        self.pop_count = 0
        self.interrupt_count = 0
        self.do_not_reevaluate_count = 0

    def pop(self):
        self.pop_count += 1

    def interrupt(self):
        self.interrupt_count += 1

    def set_do_not_reevaluate(self):
        self.do_not_reevaluate_count += 1


def make_action(cls=Idle, parameters=None):
    return cls(Blackboard(), FakeDSD(), parameters or {})


def test_name_is_class_name():
    assert make_action().name == "Idle"


def test_pop_delegates_to_dsd():
    action = make_action()
    action.pop()
    assert action._dsd.pop_count == 1


def test_interrupt_delegates_to_dsd():
    action = make_action()
    action.interrupt()
    assert action._dsd.interrupt_count == 1


def test_do_not_reevaluate_delegates_to_dsd():
    action = make_action()
    action.do_not_reevaluate()
    assert action._dsd.do_not_reevaluate_count == 1


def test_on_pop_default_is_noop():
    # The default on_pop should simply do nothing and not raise
    assert make_action().on_pop() is None


def test_publish_debug_data_stores_serializable_data():
    action = make_action()
    action.publish_debug_data("label", 42)
    assert action._debug_data == {"label": 42}


def test_publish_debug_data_wraps_non_serializable_data():
    action = make_action()

    class NotSerializable:
        def __str__(self):
            return "wrapped"

    action.publish_debug_data("label", NotSerializable())
    assert action._debug_data == {"label": "wrapped"}


def test_clear_debug_data():
    action = make_action()
    action.publish_debug_data("label", 1)
    action.clear_debug_data()
    assert action._debug_data == {}


def test_action_repr_dict():
    action = make_action()
    action.publish_debug_data("k", "v")
    assert action.repr_dict() == {"type": "action", "name": "Idle", "debug_data": {"k": "v"}}


def test_decision_repr_dict():
    decision = Chooser(Blackboard(), FakeDSD(), {})
    assert decision.repr_dict() == {"type": "decision", "name": "Chooser", "debug_data": {}}


def test_never_reevaluate_flag_defaults_false():
    assert make_action().never_reevaluate is False


def test_never_reevaluate_flag_from_short_parameter():
    assert make_action(parameters={"r": False}).never_reevaluate is True


def test_never_reevaluate_flag_from_long_parameter():
    assert make_action(parameters={"reevaluate": False}).never_reevaluate is True


def test_get_reevaluate_default_false():
    assert Chooser(Blackboard(), FakeDSD(), {}).get_reevaluate() is False


def test_get_reevaluate_true_when_flag_set():
    assert Guard(Blackboard(), FakeDSD(), {}).get_reevaluate() is True


def test_base_perform_raises_not_implemented():
    # The base perform is a safeguard for subclasses that forget to override it
    with pytest.raises(NotImplementedError):
        AbstractStackElement.perform(make_action())


def test_base_repr_dict():
    # The base implementation is normally overridden; call it directly to cover it
    assert AbstractStackElement.repr_dict(make_action()) == {
        "type": "abstract",
        "name": "Idle",
        "debug_data": {},
    }

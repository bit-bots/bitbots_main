"""Action classes for the discovery tests."""

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class FixtureActionA(AbstractActionElement):
    def perform(self, reevaluate=False):
        pass


class FixtureActionB(AbstractActionElement):
    def perform(self, reevaluate=False):
        pass


class NotAnElement:
    """A plain class that must be ignored by discovery (it is not a stack element)."""

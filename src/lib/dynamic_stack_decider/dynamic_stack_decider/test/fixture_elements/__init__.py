"""Fixture element package used by the discovery tests. Defines one action directly here so the
``__init__.py`` discovery path can be exercised."""

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class InitAction(AbstractActionElement):
    def perform(self, reevaluate=False):
        pass

"""Decision classes for the discovery tests."""

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class FixtureDecision(AbstractDecisionElement):
    def perform(self, reevaluate=False):
        return "X"

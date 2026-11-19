"""
Reusable actions, decisions, a blackboard and a DSD builder for the runtime tests.

The elements are intentionally generic and driven by the blackboard so that a wide range of
behaviors can be expressed purely through small ``.dsd`` snippets in the tests. Every element
records what it does in ``blackboard.events`` so the tests can assert on the exact execution order.
"""

import os
import tempfile
import textwrap

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from dynamic_stack_decider.dsd import DSD


class Blackboard:
    """A minimal blackboard shared between all elements of a DSD."""

    def __init__(self):
        # Ordered log of everything that happened, e.g. ("perform", "Idle") or ("decide", "Guard", True)
        self.events: list[tuple] = []
        # Maps a decision name to the result it should currently return
        self.results: dict[str, str] = {}


# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------
class _LoggingAction(AbstractActionElement):
    """Base action that logs its perform and pop calls but otherwise stays on the stack."""

    def perform(self, reevaluate=False):
        self.blackboard.events.append(("perform", self.name))

    def on_pop(self):
        self.blackboard.events.append(("pop", self.name))


class Idle(_LoggingAction):
    """A persistent action that stays on top of the stack."""


class Run(_LoggingAction):
    """A second persistent action (distinct name from Idle)."""


class Wait(_LoggingAction):
    """A third persistent action (distinct name from Idle and Run)."""


class _PoppingAction(_LoggingAction):
    """Base action that pops itself immediately after performing."""

    def perform(self, reevaluate=False):
        super().perform(reevaluate)
        return self.pop()


class StepA(_PoppingAction):
    """Pops itself immediately. Used to build sequences."""


class StepB(_PoppingAction):
    """Pops itself immediately. Used to build sequences."""


class StepC(_PoppingAction):
    """Pops itself immediately. Used to build sequences."""


class CountingAction(_LoggingAction):
    """Stays on the stack for ``count`` performs (default 1), then pops itself."""

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self._performed = 0

    def perform(self, reevaluate=False):
        self.blackboard.events.append(("perform", self.name))
        self._performed += 1
        if self._performed >= self.parameters.get("count", 1):
            return self.pop()


class Raiser(AbstractActionElement):
    """Raises an exception in its perform method (to test the DSDs error handling)."""

    def perform(self, reevaluate=False):
        self.blackboard.events.append(("perform", self.name))
        raise ValueError("boom in perform")


class Interrupter(_LoggingAction):
    """Interrupts the DSD on its first perform, then behaves like a persistent action."""

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self._interrupted = False

    def perform(self, reevaluate=False):
        self.blackboard.events.append(("perform", self.name))
        if not self._interrupted:
            self._interrupted = True
            self.interrupt()


class DoNotReevalOnce(_LoggingAction):
    """Requests ``do_not_reevaluate`` on its first perform only, then stays on the stack."""

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self._requested = False

    def perform(self, reevaluate=False):
        self.blackboard.events.append(("perform", self.name))
        if not self._requested:
            self._requested = True
            self.do_not_reevaluate()


class DoNotReevalThenPop(_LoggingAction):
    """Requests ``do_not_reevaluate`` and then pops itself (used to check the flag handling on pop)."""

    def perform(self, reevaluate=False):
        self.blackboard.events.append(("perform", self.name))
        self.do_not_reevaluate()
        return self.pop()


# ---------------------------------------------------------------------------
# Decisions
# ---------------------------------------------------------------------------
class _LoggingDecision(AbstractDecisionElement):
    """Base decision that logs its perform calls and returns ``blackboard.results[self.name]``."""

    def perform(self, reevaluate=False):
        self.blackboard.events.append(("decide", self.name, reevaluate))
        return self.blackboard.results[self.name]


class Chooser(_LoggingDecision):
    """A decision that is NOT reevaluated while it is in the middle of the stack."""


class Guard(_LoggingDecision):
    """A decision that IS reevaluated while it is in the middle of the stack."""

    _reevaluate = True


class ElseGuard(_LoggingDecision):
    """A reevaluating decision, typically used together with an ELSE branch."""

    _reevaluate = True


class ReturnNone(_LoggingDecision):
    """A decision whose perform returns None (to test the 'forgot to return' safeguard)."""

    def perform(self, reevaluate=False):
        self.blackboard.events.append(("decide", self.name, reevaluate))
        return None


class PopOnReeval(_LoggingDecision):
    """A reevaluating decision that pops itself while being reevaluated."""

    _reevaluate = True

    def perform(self, reevaluate=False):
        self.blackboard.events.append(("decide", self.name, reevaluate))
        if reevaluate:
            self.pop()
        return self.blackboard.results[self.name]


class InterruptOnReeval(_LoggingDecision):
    """A reevaluating decision that interrupts the DSD while being reevaluated."""

    _reevaluate = True

    def perform(self, reevaluate=False):
        self.blackboard.events.append(("decide", self.name, reevaluate))
        if reevaluate:
            self.interrupt()
        return self.blackboard.results[self.name]


ACTIONS = {
    cls.__name__: cls
    for cls in [
        Idle,
        Run,
        Wait,
        StepA,
        StepB,
        StepC,
        CountingAction,
        Raiser,
        Interrupter,
        DoNotReevalOnce,
        DoNotReevalThenPop,
    ]
}

DECISIONS = {
    cls.__name__: cls
    for cls in [
        Chooser,
        Guard,
        ElseGuard,
        ReturnNone,
        PopOnReeval,
        InterruptOnReeval,
    ]
}


def make_dsd(dsd_text, blackboard=None, node=None, debug_topic=None):
    """
    Build a DSD from an inline ``.dsd`` description, using the elements defined in this module.

    The description is dedented so that it can be written as a nicely indented multiline string in
    the tests. It is then written to a temporary file, parsed and bound. The DSD is returned with the
    start element on the stack but is not yet updated, mirroring real usage.
    """
    blackboard = blackboard if blackboard is not None else Blackboard()
    dsd = DSD(blackboard, debug_topic=debug_topic, node=node)
    dsd.actions = dict(ACTIONS)
    dsd.decisions = dict(DECISIONS)

    with tempfile.NamedTemporaryFile("w", suffix=".dsd", delete=False) as f:
        f.write(textwrap.dedent(dsd_text))
        path = f.name
    try:
        dsd.load_behavior(path)
    finally:
        os.unlink(path)
    return dsd


def stack_names(dsd):
    """Return the class names of the instances currently on the stack (bottom to top)."""
    return [instance.name for _, instance in dsd.stack]

"""
Runtime behavior tests for the DSD execution engine (``dsd.py``).

These tests drive a DSD through ``update()`` and assert on the resulting stack and execution order.
They aim to cover every runtime code path: pushing/popping, sequences (chains), reevaluation
(including ELSE handling), the ``never_reevaluate`` / ``do_not_reevaluate`` flags, interrupts,
decisions and subtrees following actions, error handling and the debug publishing.
"""

import json

import pytest

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement

from .runtime_elements import Blackboard, make_dsd, stack_names


class FakePublisher:
    """Records published messages and reports a configurable subscription count."""

    def __init__(self):
        self.messages = []
        self.subscription_count = 1

    def publish(self, msg):
        self.messages.append(msg)

    def get_subscription_count(self):
        return self.subscription_count


class FakeNode:
    """A stand-in for an rclpy node that just hands out FakePublishers."""

    def __init__(self):
        self.publishers = {}

    def create_publisher(self, msg_type, topic, qos):
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher


# ---------------------------------------------------------------------------
# Basic decision / action flow
# ---------------------------------------------------------------------------
def test_start_element_on_stack_after_load():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    # After loading, only the start element is on the stack and nothing has been performed yet
    assert stack_names(dsd) == ["Chooser"]
    assert dsd.blackboard.events == []


def test_decision_pushes_action():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()

    assert stack_names(dsd) == ["Chooser", "Idle"]
    assert dsd.blackboard.events == [("decide", "Chooser", False), ("perform", "Idle")]


def test_persistent_action_stays_on_stack():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    for _ in range(3):
        dsd.update()

    assert stack_names(dsd) == ["Chooser", "Idle"]
    # The action was performed on every update
    assert dsd.blackboard.events.count(("perform", "Idle")) == 3


def test_action_pops_and_decision_redecides():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @StepA
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()
    # StepA popped itself again, so only the decision remains
    assert stack_names(dsd) == ["Chooser"]
    assert ("pop", "StepA") in dsd.blackboard.events

    dsd.update()
    # The decision pushes and runs StepA again
    assert dsd.blackboard.events.count(("perform", "StepA")) == 2


def test_nested_decisions():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            DOWN --> $Guard
                GO --> @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "DOWN"
    dsd.blackboard.results["Guard"] = "GO"

    dsd.update()

    assert stack_names(dsd) == ["Chooser", "Guard", "Idle"]


# ---------------------------------------------------------------------------
# Sequences (chains via next_element)
# ---------------------------------------------------------------------------
def test_sequence_advances_one_action_per_tick():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @StepA, @StepB, @StepC
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()
    # StepA performed and popped, advancing to StepB (not yet performed)
    assert stack_names(dsd) == ["Chooser", "StepB"]

    dsd.update()
    assert stack_names(dsd) == ["Chooser", "StepC"]

    dsd.update()
    # The last action has no successor, so it pops normally and the decision remains
    assert stack_names(dsd) == ["Chooser"]

    # Every action was performed and popped exactly once, in order
    assert [e for e in dsd.blackboard.events if e[0] == "pop"] == [
        ("pop", "StepA"),
        ("pop", "StepB"),
        ("pop", "StepC"),
    ]


def test_sequence_with_multi_tick_action():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @CountingAction + count:2, @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()
    # CountingAction needs two performs before it advances
    assert stack_names(dsd) == ["Chooser", "CountingAction"]

    dsd.update()
    # After the second perform it popped and advanced to Idle
    assert stack_names(dsd) == ["Chooser", "Idle"]
    assert dsd.blackboard.events.count(("perform", "CountingAction")) == 2


def test_decision_after_action():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @StepA, $Guard
                ON --> @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"
    dsd.blackboard.results["Guard"] = "ON"

    dsd.update()
    # StepA ran and advanced to the decision Guard
    assert stack_names(dsd) == ["Chooser", "Guard"]

    dsd.update()
    # Guard now decides and pushes its child
    assert stack_names(dsd) == ["Chooser", "Guard", "Idle"]


def test_subtree_after_action():
    dsd = make_dsd(
        """
        #Sub
        $Guard
            ON --> @Idle
        -->T
        $Chooser
            GO --> @StepA, #Sub
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"
    dsd.blackboard.results["Guard"] = "ON"

    dsd.update()
    assert stack_names(dsd) == ["Chooser", "Guard"]

    dsd.update()
    assert stack_names(dsd) == ["Chooser", "Guard", "Idle"]


# ---------------------------------------------------------------------------
# Reevaluation
# ---------------------------------------------------------------------------
def test_reevaluation_rebuilds_stack_on_change():
    dsd = make_dsd(
        """
        -->T
        $Guard
            A --> @Idle
            B --> @Run
        """
    )
    dsd.blackboard.results["Guard"] = "A"

    dsd.update()
    assert stack_names(dsd) == ["Guard", "Idle"]

    # Same result -> no rebuild
    dsd.update()
    assert stack_names(dsd) == ["Guard", "Idle"]

    # Changed result -> the stack above the decision is dropped and rebuilt
    dsd.blackboard.results["Guard"] = "B"
    dsd.update()
    assert stack_names(dsd) == ["Guard", "Run"]
    assert ("pop", "Idle") in dsd.blackboard.events


def test_reevaluation_reruns_only_reevaluating_decisions():
    # Guard reevaluates, the nested Chooser does not
    dsd = make_dsd(
        """
        -->T
        $Guard
            GO --> $Chooser
                A --> @Idle
                B --> @Run
        """
    )
    dsd.blackboard.results["Guard"] = "GO"
    dsd.blackboard.results["Chooser"] = "A"

    dsd.update()
    assert stack_names(dsd) == ["Guard", "Chooser", "Idle"]

    # Changing the non-reevaluating Chooser has no effect while it is in the middle of the stack
    dsd.blackboard.results["Chooser"] = "B"
    dsd.update()
    assert stack_names(dsd) == ["Guard", "Chooser", "Idle"]


def test_reevaluation_else_branch():
    dsd = make_dsd(
        """
        -->T
        $ElseGuard
            A --> @Idle
            ELSE --> @Run
        """
    )
    dsd.blackboard.results["ElseGuard"] = "A"

    dsd.update()
    assert stack_names(dsd) == ["ElseGuard", "Idle"]

    # An unknown result maps to the ELSE branch -> rebuild into the ELSE child
    dsd.blackboard.results["ElseGuard"] = "SOMETHING_ELSE"
    dsd.update()
    assert stack_names(dsd) == ["ElseGuard", "Run"]

    # Still an ELSE result -> no rebuild (the activation reason stays "ELSE")
    dsd.blackboard.results["ElseGuard"] = "YET_ANOTHER"
    dsd.update()
    assert stack_names(dsd) == ["ElseGuard", "Run"]
    assert dsd.blackboard.events.count(("pop", "Run")) == 0

    # Back to a concrete result -> rebuild into the concrete child
    dsd.blackboard.results["ElseGuard"] = "A"
    dsd.update()
    assert stack_names(dsd) == ["ElseGuard", "Idle"]


# ---------------------------------------------------------------------------
# never_reevaluate / do_not_reevaluate
# ---------------------------------------------------------------------------
def test_never_reevaluate_parameter_suppresses_reevaluation():
    dsd = make_dsd(
        """
        -->T
        $Guard
            A --> @Idle + r:false
            B --> @Run
        """
    )
    dsd.blackboard.results["Guard"] = "A"

    dsd.update()
    assert stack_names(dsd) == ["Guard", "Idle"]

    # Because Idle has never_reevaluate set, the guard is not reevaluated and the change is ignored
    dsd.blackboard.results["Guard"] = "B"
    for _ in range(3):
        dsd.update()
    assert stack_names(dsd) == ["Guard", "Idle"]


def test_do_not_reevaluate_delays_reevaluation_by_one_tick():
    dsd = make_dsd(
        """
        -->T
        $Guard
            A --> @DoNotReevalOnce
            B --> @Run
        """
    )
    dsd.blackboard.results["Guard"] = "A"

    dsd.update()
    assert stack_names(dsd) == ["Guard", "DoNotReevalOnce"]

    # The action requested do_not_reevaluate once, so this update skips reevaluation
    dsd.blackboard.results["Guard"] = "B"
    dsd.update()
    assert stack_names(dsd) == ["Guard", "DoNotReevalOnce"]

    # The flag was reset, so the next update reevaluates and rebuilds
    dsd.update()
    assert stack_names(dsd) == ["Guard", "Run"]


def test_do_not_reevaluate_reset_on_normal_pop():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @DoNotReevalThenPop
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()
    # The action set do_not_reevaluate but then popped (no successor), so the flag is reset
    assert dsd.do_not_reevaluate is False
    assert stack_names(dsd) == ["Chooser"]


def test_do_not_reevaluate_preserved_across_sequence_advance():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @DoNotReevalThenPop, @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()
    # The action advanced to the next chain element and kept do_not_reevaluate set
    assert stack_names(dsd) == ["Chooser", "Idle"]
    assert dsd.do_not_reevaluate is True


# ---------------------------------------------------------------------------
# Interrupts
# ---------------------------------------------------------------------------
def test_external_interrupt_resets_stack():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()
    assert stack_names(dsd) == ["Chooser", "Idle"]
    top_before = dsd.stack[0][1]

    dsd.interrupt()
    assert stack_names(dsd) == ["Chooser"]
    # A fresh start element instance is created
    assert dsd.stack[0][1] is not top_before


def test_interrupt_from_within_an_action():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Interrupter
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()
    # The action interrupted the DSD, clearing the stack back to the start element
    assert stack_names(dsd) == ["Chooser"]


def test_interrupt_during_reevaluation():
    dsd = make_dsd(
        """
        -->T
        $InterruptOnReeval
            GO --> @Idle
        """
    )
    dsd.blackboard.results["InterruptOnReeval"] = "GO"

    dsd.update()
    assert stack_names(dsd) == ["InterruptOnReeval", "Idle"]

    # On the next update the root decision is reevaluated and interrupts, resetting the stack
    dsd.update()
    assert stack_names(dsd) == ["InterruptOnReeval"]


# ---------------------------------------------------------------------------
# pop edge cases
# ---------------------------------------------------------------------------
def test_pop_of_only_element_is_noop():
    # A behavior whose root is a self-popping action
    dsd = make_dsd(
        """
        -->T
        @StepA
        """
    )

    dsd.update()
    # The action tried to pop itself, but it is the only element, so the stack is unchanged
    assert stack_names(dsd) == ["StepA"]
    assert ("pop", "StepA") not in dsd.blackboard.events


def test_reevaluation_pop_at_root_does_not_shorten_stack():
    # A reevaluating root decision that pops itself during reevaluation. Because it is at the bottom
    # of the stack, no element above the reevaluation index is dropped and the update returns early.
    dsd = make_dsd(
        """
        -->T
        $PopOnReeval
            GO --> @Idle
        """
    )
    dsd.blackboard.results["PopOnReeval"] = "GO"

    dsd.update()
    assert stack_names(dsd) == ["PopOnReeval", "Idle"]

    performed_before = dsd.blackboard.events.count(("perform", "Idle"))
    dsd.update()
    # The stack is untouched and, because reevaluation stopped, the top action was not performed again
    assert stack_names(dsd) == ["PopOnReeval", "Idle"]
    assert dsd.blackboard.events.count(("perform", "Idle")) == performed_before
    assert dsd.stack_reevaluate is False


def test_reevaluation_pop_in_the_middle_shortens_stack():
    # A reevaluating decision that is not at the bottom pops itself during reevaluation. This shortens
    # the stack down to the reevaluation index. The engine then swallows the resulting internal error
    # (this is an unusual edge case); the important contract is that update() does not raise and the
    # stack ends up shortened.
    dsd = make_dsd(
        """
        -->T
        $Guard
            GO --> $PopOnReeval
                X --> @Idle
        """
    )
    dsd.blackboard.results["Guard"] = "GO"
    dsd.blackboard.results["PopOnReeval"] = "X"

    dsd.update()
    assert stack_names(dsd) == ["Guard", "PopOnReeval", "Idle"]

    dsd.update()
    # PopOnReeval and its child were dropped during reevaluation
    assert stack_names(dsd) == ["Guard"]
    assert ("pop", "Idle") in dsd.blackboard.events


# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------
def test_exception_in_perform_is_swallowed():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Raiser
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    # The exception raised inside the action must not propagate out of update()
    dsd.update()
    assert stack_names(dsd) == ["Chooser", "Raiser"]
    assert ("perform", "Raiser") in dsd.blackboard.events


def test_unknown_decision_result_is_swallowed():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "UNKNOWN"

    # get_child raises a KeyError which is caught inside update()
    dsd.update()
    assert stack_names(dsd) == ["Chooser"]


def test_decision_returning_none_exits():
    dsd = make_dsd(
        """
        -->T
        $ReturnNone
            GO --> @Idle
        """
    )

    # Returning None from a decision is a programming error and triggers sys.exit
    with pytest.raises(SystemExit):
        dsd.update()


# ---------------------------------------------------------------------------
# _bind_modules error paths
# ---------------------------------------------------------------------------
def test_unknown_action_name_raises_on_load():
    with pytest.raises(AssertionError):
        make_dsd(
            """
            -->T
            $Chooser
                GO --> @DoesNotExist
            """
        )


def test_unknown_decision_name_raises_on_load():
    with pytest.raises(AssertionError):
        make_dsd(
            """
            -->T
            $DoesNotExist
                GO --> @Idle
            """
        )


def test_bind_modules_rejects_unknown_element_type():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )

    class Bogus:
        pass

    with pytest.raises(ValueError):
        dsd._bind_modules(Bogus())


# ---------------------------------------------------------------------------
# Misc DSD API
# ---------------------------------------------------------------------------
def test_get_stack_returns_the_stack():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    assert dsd.get_stack() is dsd.stack


def test_set_do_not_reevaluate():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    assert dsd.do_not_reevaluate is False
    dsd.set_do_not_reevaluate()
    assert dsd.do_not_reevaluate is True


def test_stack_holds_bound_instances():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"
    dsd.update()

    root_tree, root_instance = dsd.stack[0]
    assert isinstance(root_instance, AbstractDecisionElement)
    top_tree, top_instance = dsd.stack[-1]
    assert isinstance(top_instance, AbstractActionElement)
    # The instances share the blackboard
    assert root_instance.blackboard is dsd.blackboard is top_instance.blackboard


# ---------------------------------------------------------------------------
# Debug publishing
# ---------------------------------------------------------------------------
def test_debug_tree_published_on_load():
    node = FakeNode()
    make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """,
        node=node,
        debug_topic="dbg",
    )

    tree_messages = node.publishers["dbg/dsd_tree"].messages
    assert len(tree_messages) == 1
    tree = json.loads(tree_messages[0].data)
    assert tree["type"] == "decision"
    assert tree["name"] == "Chooser"
    assert "id" in tree
    assert tree["children"]["GO"]["type"] == "action"


def test_debug_stack_and_current_action_published():
    node = FakeNode()
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """,
        node=node,
        debug_topic="dbg",
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()

    stack_messages = node.publishers["dbg/dsd_stack"].messages
    assert stack_messages
    stack = json.loads(stack_messages[-1].data)
    # Bottom of the stack is the decision, above it the action, matched via the stable id
    assert stack["type"] == "decision" and "id" in stack
    assert stack["next"]["type"] == "action"
    assert stack["next"]["name"] == "Idle"
    assert stack["next"]["next"] is None

    # The current action is published exactly once (it does not change while Idle stays on top)
    action_messages = node.publishers["dbg/dsd_current_action"].messages
    dsd.update()
    dsd.update()
    assert [m.data for m in action_messages] == ["Idle"]


def test_debug_stack_not_published_without_subscribers():
    node = FakeNode()
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """,
        node=node,
        debug_topic="dbg",
    )
    dsd.blackboard.results["Chooser"] = "GO"
    node.publishers["dbg/dsd_stack"].subscription_count = 0

    dsd.update()

    assert node.publishers["dbg/dsd_stack"].messages == []


def test_fresh_blackboard_between_dsds():
    bb1 = Blackboard()
    bb2 = Blackboard()
    dsd1 = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """,
        blackboard=bb1,
    )
    dsd2 = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Run
        """,
        blackboard=bb2,
    )
    dsd1.blackboard.results["Chooser"] = "GO"
    dsd2.blackboard.results["Chooser"] = "GO"

    dsd1.update()
    assert ("perform", "Idle") in bb1.events
    assert bb2.events == []

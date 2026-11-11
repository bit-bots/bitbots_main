"""
Unit tests for the tree element classes (``tree.py``): child lookup (including ELSE and error cases),
activation-reason propagation along a chain, and the JSON representation (``repr_dict``) including the
stable ``id`` and the ``next`` chain link.
"""

import pytest

from dynamic_stack_decider.tree import ActionTreeElement, DecisionTreeElement, Tree


# ---------------------------------------------------------------------------
# get_child
# ---------------------------------------------------------------------------
def test_get_child_returns_matching_child():
    d = DecisionTreeElement("D", None)
    a = ActionTreeElement("A", d)
    d.add_child_element(a, "RESULT")
    assert d.get_child("RESULT") is a
    assert a.activation_reason == "RESULT"


def test_get_child_falls_back_to_else():
    d = DecisionTreeElement("D", None)
    a = ActionTreeElement("A", d)
    d.add_child_element(a, "ELSE")
    assert d.get_child("ANY_UNMAPPED_RESULT") is a


def test_get_child_unknown_result_raises_keyerror():
    d = DecisionTreeElement("D", None)
    d.add_child_element(ActionTreeElement("A", d), "X")
    with pytest.raises(KeyError):
        d.get_child("Y")


def test_get_child_none_result_exits():
    d = DecisionTreeElement("D", None)
    with pytest.raises(SystemExit):
        d.get_child(None)


def test_action_get_child_is_always_none():
    a = ActionTreeElement("A", None)
    assert a.get_child("anything") is None


# ---------------------------------------------------------------------------
# Activation reason propagation
# ---------------------------------------------------------------------------
def test_set_activation_reason_propagates_along_chain():
    a = ActionTreeElement("A", None)
    b = ActionTreeElement("B", a)
    c = ActionTreeElement("C", b)
    a.next_element = b
    b.next_element = c

    a.set_activation_reason("REASON")

    assert a.activation_reason == "REASON"
    assert b.activation_reason == "REASON"
    assert c.activation_reason == "REASON"


def test_set_activation_reason_does_not_override_decision_children():
    # A chain ending in a decision: the decision gets the chain's reason, but its children keep theirs
    a = ActionTreeElement("A", None)
    d = DecisionTreeElement("D", a)
    child = ActionTreeElement("child", d)
    d.add_child_element(child, "C")
    a.next_element = d

    a.set_activation_reason("REASON")

    assert d.activation_reason == "REASON"
    assert child.activation_reason == "C"


def test_add_child_element_propagates_reason_to_chain():
    d = DecisionTreeElement("D", None)
    head = ActionTreeElement("head", d)
    tail = ActionTreeElement("tail", head)
    head.next_element = tail

    d.add_child_element(head, "R")

    assert head.activation_reason == "R"
    assert tail.activation_reason == "R"


# ---------------------------------------------------------------------------
# repr_dict
# ---------------------------------------------------------------------------
def test_action_repr_dict_contains_id_type_and_next():
    a = ActionTreeElement("A", None, parameters={"k": "v"})
    b = ActionTreeElement("B", a)
    a.next_element = b

    r = a.repr_dict()

    assert r["type"] == "action"
    assert r["name"] == "A"
    assert r["parameters"] == {"k": "v"}
    assert "id" in r
    assert r["next"]["name"] == "B"
    assert r["next"]["next"] is None


def test_action_repr_dict_next_is_none_without_successor():
    assert ActionTreeElement("A", None).repr_dict()["next"] is None


def test_decision_repr_dict_contains_children_and_id():
    d = DecisionTreeElement("D", None)
    d.add_child_element(ActionTreeElement("A", d), "R")

    r = d.repr_dict()

    assert r["type"] == "decision"
    assert "id" in r
    assert r["children"]["R"]["name"] == "A"


def test_repr_dict_id_is_stable_and_unique():
    a1 = ActionTreeElement("A", None)
    a2 = ActionTreeElement("A", None)
    assert a1.repr_dict()["id"] == a1.repr_dict()["id"]
    assert a1.repr_dict()["id"] != a2.repr_dict()["id"]


# ---------------------------------------------------------------------------
# Tree and string representations
# ---------------------------------------------------------------------------
def test_tree_repr_dict_delegates_to_root():
    d = DecisionTreeElement("D", None)
    d.add_child_element(ActionTreeElement("A", d), "R")
    tree = Tree()
    tree.set_root_element(d)

    assert tree.repr_dict()["name"] == "D"
    assert "D" in repr(tree)


def test_str_returns_name():
    assert str(ActionTreeElement("A", None)) == "A"
    assert str(DecisionTreeElement("D", None)) == "D"


def test_action_repr_includes_successor():
    a = ActionTreeElement("A", None)
    a.next_element = ActionTreeElement("B", a)
    assert "A" in repr(a) and "B" in repr(a)

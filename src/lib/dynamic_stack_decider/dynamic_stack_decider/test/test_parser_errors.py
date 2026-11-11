"""
Tests for the parser's error handling and less common code paths: malformed indentation, invalid
element/parameter syntax, invalid sequences and subtree references, and node-provided (``%``)
parameters.
"""

import os
import tempfile

import pytest

from dynamic_stack_decider.parser import DsdParser, ParseError


def parse(text, node=None):
    with tempfile.NamedTemporaryFile("w", suffix=".dsd", delete=False) as f:
        f.write(text)
        path = f.name
    try:
        return DsdParser(node).parse(path)
    finally:
        os.unlink(path)


def test_indent_not_multiple_of_four():
    with pytest.raises(ParseError, match="Indent is not a multiple of 4"):
        parse("-->T\n$Chooser\n   GO --> @FirstAction\n")


def test_element_neither_action_nor_decision():
    with pytest.raises(ParseError, match="neither an action nor a decision"):
        parse("-->T\n$Chooser\n    GO --> FirstAction\n")


def test_root_element_must_start_with_marker():
    with pytest.raises(ParseError, match="start with either"):
        parse("-->T\nFirstDecision\n")


def test_invalid_parameter_list():
    with pytest.raises(ParseError, match="Invalid parameter list"):
        parse("-->T\n$Chooser\n    GO --> @FirstAction + keywithoutvalue\n")


def test_parameter_value_with_space():
    with pytest.raises(ParseError, match="should not contain spaces"):
        parse("-->T\n$Chooser\n    GO --> @FirstAction + key:val ue\n")


def test_decision_not_last_in_sequence():
    with pytest.raises(ParseError, match="Only actions may be followed"):
        parse("-->T\n$Chooser\n    GO --> @FirstAction, $Guard, @SecondAction\n")


def test_subtree_not_defined():
    with pytest.raises(ParseError, match="not defined"):
        parse("-->T\n$Chooser\n    GO --> #DoesNotExist\n")


def test_subtree_invalid_parameters():
    with pytest.raises(ParseError, match="Invalid parameters specified"):
        parse("#Sub + key1\n@FirstAction + p:*key1\n-->T\n$Chooser\n    GO --> #Sub + wrongkey:value\n")


def test_subtree_unknown_reference():
    with pytest.raises(ParseError, match="Unknown reference"):
        parse(
            # The subtree declares parameter 'key1' but the action references the unknown '*missing'
            "#Sub + key1\n@FirstAction + p:*missing\n-->T\n$Chooser\n    GO --> #Sub + key1:value\n"
        )


def test_subtree_used_as_root_of_another_subtree():
    # When a subtree body starts by referencing another subtree, that subtree becomes its root
    tree = parse("#Inner\n@FirstAction\n#Outer\nFOO --> #Inner\n-->T\n$Chooser\n    GO --> #Outer\n")
    assert tree.root_element.get_child("GO").name == "FirstAction"


class _FakeParameter:
    def __init__(self, value):
        self.value = value


class _FakeNode:
    def __init__(self, values):
        self._values = values

    def get_parameter(self, name):
        return _FakeParameter(self._values[name])


def test_node_provided_parameter():
    node = _FakeNode({"speed": 5})
    tree = parse("-->T\n$Chooser\n    GO --> @FirstAction + key:%speed\n", node=node)
    assert tree.root_element.get_child("GO").parameters == {"key": 5}


def test_node_provided_parameter_without_node_is_none():
    tree = parse("-->T\n$Chooser\n    GO --> @FirstAction + key:%speed\n", node=None)
    assert tree.root_element.get_child("GO").parameters == {"key": None}

import os

import pytest

from dynamic_stack_decider.parser import DsdParser, Tree
from dynamic_stack_decider.tree import ActionTreeElement, DecisionTreeElement, SequenceTreeElement


@pytest.fixture()
def dsd_tree():
    parser = DsdParser()
    yield parser.parse(os.path.join(os.path.dirname(__file__), "test.dsd"))


def test_root_element(dsd_tree: Tree):
    root_element = dsd_tree.root_element
    assert isinstance(root_element, DecisionTreeElement)
    assert root_element.name == "FirstDecision"


def test_possible_results(dsd_tree: Tree):
    assert set(dsd_tree.root_element.children.keys()) == {
        "ACTION",
        "DECISION",
        "SUBBEHAVIOR",
        "SEQUENCE",
        "PARAMETERS",
        "LINE_COMMENT",
        "BLOCK_COMMENT",
        "COMPLICATED_COMMENT",
        "MULTIPLE_PARAMETERS",
        "SECOND_SUBBEHAVIOR_1",
        "SECOND_SUBBEHAVIOR_2",
        "PARAMETER_DECISION",
        "PARAMETER_SUBBEHAVIOR",
        "NESTED_PARAMETER_SUBBEHAVIOR",
        "SEQUENCE_TREE",
    }


def test_following_elements(dsd_tree: Tree):
    first_child = dsd_tree.root_element.get_child("ACTION")
    assert first_child.name == "FirstAction"
    assert isinstance(first_child, ActionTreeElement)

    second_child = dsd_tree.root_element.get_child("DECISION")
    assert second_child.name == "SecondDecision"
    assert isinstance(second_child, DecisionTreeElement)


def test_nested_decision(dsd_tree: Tree):
    decision_child = dsd_tree.root_element.get_child("DECISION")
    assert set(decision_child.children.keys()) == {"FIRST", "SECOND"}
    assert decision_child.get_child("FIRST").name == "FirstAction"
    assert isinstance(decision_child.get_child("FIRST"), ActionTreeElement)
    assert decision_child.get_child("SECOND").name == "SecondAction"
    assert isinstance(decision_child.get_child("SECOND"), ActionTreeElement)


def test_sub_behavior(dsd_tree: Tree):
    sub_behavior_root_decision = dsd_tree.root_element.get_child("SUBBEHAVIOR")
    assert sub_behavior_root_decision.name == "ThirdDecision"
    assert set(sub_behavior_root_decision.children.keys()) == {"FIRST", "SECOND"}
    assert sub_behavior_root_decision.get_child("FIRST").name == "FirstAction"
    assert isinstance(sub_behavior_root_decision.get_child("FIRST"), ActionTreeElement)
    assert sub_behavior_root_decision.get_child("SECOND").name == "SecondAction"
    assert isinstance(sub_behavior_root_decision.get_child("SECOND"), ActionTreeElement)


def test_sequence_element(dsd_tree: Tree):
    sequence_element = dsd_tree.root_element.get_child("SEQUENCE")
    assert isinstance(sequence_element, SequenceTreeElement)
    assert len(sequence_element.action_elements) == 2
    first_action = sequence_element.action_elements[0]
    assert first_action.name == "FirstAction"
    assert isinstance(first_action, ActionTreeElement)
    second_action = sequence_element.action_elements[1]
    assert second_action.name == "SecondAction"
    assert isinstance(second_action, ActionTreeElement)


def test_action_parameters(dsd_tree: Tree):
    parameter_element = dsd_tree.root_element.get_child("PARAMETERS")
    assert parameter_element.name == "FirstAction"
    assert isinstance(parameter_element, ActionTreeElement)
    assert parameter_element.parameters == {"key": "value"}


def test_decision_parameters(dsd_tree: Tree):
    parameter_element = dsd_tree.root_element.get_child("PARAMETER_DECISION")
    assert parameter_element.name == "FirstDecision"
    assert isinstance(parameter_element, DecisionTreeElement)
    assert parameter_element.parameters == {"key": "value"}


def test_line_comment(dsd_tree: Tree):
    comment_element = dsd_tree.root_element.get_child("LINE_COMMENT")
    assert comment_element.name == "FirstAction"
    assert isinstance(comment_element, ActionTreeElement)


def test_block_comment(dsd_tree: Tree):
    comment_element = dsd_tree.root_element.get_child("BLOCK_COMMENT")
    assert comment_element.name == "FirstAction"
    assert isinstance(comment_element, ActionTreeElement)
    assert comment_element.parameters == {"key": "value"}


def test_complicated_comment(dsd_tree: Tree):
    comment_element = dsd_tree.root_element.get_child("COMPLICATED_COMMENT")
    assert comment_element.name == "FirstAction"
    assert isinstance(comment_element, ActionTreeElement)


def test_multiple_parameters(dsd_tree: Tree):
    parameter_element = dsd_tree.root_element.get_child("MULTIPLE_PARAMETERS")
    assert parameter_element.name == "FirstAction"
    assert isinstance(parameter_element, ActionTreeElement)
    assert parameter_element.parameters == {"key1": "value1", "key2": "value2"}


def test_multiple_subbehavior_references(dsd_tree: Tree):
    sub_behavior_1_root_decision = dsd_tree.root_element.get_child("SECOND_SUBBEHAVIOR_1")
    sub_behavior_2_root_decision = dsd_tree.root_element.get_child("SECOND_SUBBEHAVIOR_2")
    assert sub_behavior_1_root_decision.name == sub_behavior_2_root_decision.name
    assert sub_behavior_1_root_decision.activation_reason == "SECOND_SUBBEHAVIOR_1"
    assert sub_behavior_2_root_decision.activation_reason == "SECOND_SUBBEHAVIOR_2"


def test_parameter_subbehavior(dsd_tree: Tree):
    parameter_subbehavior_decision = dsd_tree.root_element.get_child("PARAMETER_SUBBEHAVIOR")
    first_action = parameter_subbehavior_decision.get_child("FIRST")
    assert first_action.parameters == {"key": "value1"}
    action_sequence = parameter_subbehavior_decision.get_child("SECOND")
    assert action_sequence.action_elements[0].parameters == {"key": "value2"}
    assert action_sequence.action_elements[1].parameters == {"key": "value2"}


def test_nested_parameter_subbehavior(dsd_tree: Tree):
    subbehavior_decision = dsd_tree.root_element.get_child("NESTED_PARAMETER_SUBBEHAVIOR")
    inner_subbehavior_decision = subbehavior_decision.get_child("FIRST")
    first_action = inner_subbehavior_decision.get_child("FIRST")
    assert first_action.parameters == {"key": "nested1"}
    action_sequence = inner_subbehavior_decision.get_child("SECOND")
    assert action_sequence.action_elements[0].parameters == {"key": "nested2"}
    assert action_sequence.action_elements[1].parameters == {"key": "nested2"}


def test_sequence_tree(dsd_tree: Tree):
    sequence_tree = dsd_tree.root_element.get_child("SEQUENCE_TREE")
    assert isinstance(sequence_tree, SequenceTreeElement)
    assert sequence_tree.action_elements[0].name == "FirstAction"
    assert sequence_tree.action_elements[1].name == "SecondAction"

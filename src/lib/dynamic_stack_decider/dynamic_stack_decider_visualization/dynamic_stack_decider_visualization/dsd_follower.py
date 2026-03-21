import json
import uuid
from typing import Optional, Union

import pydot
from PyQt5.QtGui import QStandardItem, QStandardItemModel
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class ParseError(Exception):
    pass


class DsdFollower:
    def __init__(self, node: Node, debug_topic: str):
        """
        Creates a new DSD follower which subscribes to the given debug topics,
        fetches it's tree and listens for the current stack.
        It provides the rendered tree and the current stack visualization based on the received data.

        :param node: Reference to the used ROS node\
        :param debug_topic: The topic namespace on which the DSD publishes its debug data
        """
        self._node = node

        self.dsd_debug_topic = debug_topic

        self.tree: Optional[dict] = None
        self.stack: Optional[dict] = None

        self._cached_dotgraph: Optional[pydot.Dot] = None

        # Subscribe to the DSDs tree (latched)
        dsd_tree_topic = f"{debug_topic}/dsd_tree"
        self.tree_sub = self._node.create_subscription(
            String,
            dsd_tree_topic,
            self._tree_callback,
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._node.get_logger().info(f"Subscribed to {dsd_tree_topic}")

        # Subscribe to the DSDs stack
        dsd_stack_topic = f"{debug_topic}/dsd_stack"
        self.stack_sub = self._node.create_subscription(String, dsd_stack_topic, self._stack_callback, 10)
        self._node.get_logger().info(f"Subscribed to {dsd_stack_topic}")

    def _tree_callback(self, msg):
        self._node.get_logger().info("Received tree")
        # Deserialize the tree message
        # This is a json string because the tree is a dynamically nested structure of variable length
        self.tree = json.loads(msg.data)

    def _stack_callback(self, msg):
        # Abort if no tree was received yet
        if self.tree is None:
            return

        # Deserialize the stack message
        stack = json.loads(msg.data)

        # Check if the stack changed (ignore debug_data log messages)
        if DsdFollower.stack_dict_changed(self.stack, stack, ignore_keys=["debug_data"]):
            # Reset cache / trigger redraw
            self.reset_cache()

        # Update stack
        self.stack = stack

    def reset_cache(self):
        """Reset the cached dotgraph"""
        self._cached_dotgraph = None

    @staticmethod
    def stack_dict_changed(old_stack_element, new_stack_element, ignore_keys) -> bool:
        """
        Check if two stack dicts are different
        """
        if type(old_stack_element) != type(new_stack_element):
            return True
        elif isinstance(old_stack_element, dict):
            # Check if the length of the stack changed
            if len(old_stack_element) != len(new_stack_element):
                return True

            # Check if the content of the stack changed
            for key in old_stack_element.keys():
                if key in ignore_keys:
                    continue
                if key not in new_stack_element.keys():
                    return True
                if DsdFollower.stack_dict_changed(old_stack_element[key], new_stack_element[key], ignore_keys):
                    return True
        elif isinstance(old_stack_element, list):
            # Check if the length of the stack changed
            if len(old_stack_element) != len(new_stack_element):
                return True
            # Check if the content of the stack changed
            for i in range(len(old_stack_element)):
                if DsdFollower.stack_dict_changed(old_stack_element[i], new_stack_element[i], ignore_keys):
                    return True
        elif isinstance(old_stack_element, (int, float, str, bool)) or old_stack_element is None:
            if old_stack_element != new_stack_element:
                return True
        else:
            raise ParseError(f"Unknown type {type(old_stack_element)}")
        # The stack did not change
        return False

    @staticmethod
    def _error_dotgraph():
        dot = pydot.Dot(graph_type="digraph")

        uid1 = str(uuid.uuid4())
        dot.add_node(pydot.Node(uid1, label="I have not received anything from the dsd yet"))

        uid2 = str(uuid.uuid4())
        dot.add_node(
            pydot.Node(
                uid2,
                label="Please make sure that\n"
                "- The appropriate dsd is started\n"
                "- You are connected to the same roscore\n"
                "- param /debug_active is True",
            )
        )

        dot.add_edge(pydot.Edge(uid1, uid2))

        return dot

    @staticmethod
    def _empty_item_model():
        return QStandardItemModel()

    @staticmethod
    def _dot_node_from_tree_element(tree_element: dict, stack_element: Optional[dict] = None) -> pydot.Node:
        """
        :param tree_element: The node from the dsd tree which should be converted to a dot node
        :param stack_element: The corresponding element from the stack
        :return: The corresponding dot node
        """

        def param_string(params: dict) -> str:
            """
            :param params: A dict of parameters
            :return: A string representation of the parameters
            """
            # Return empty string if no parameters are given
            if not params:
                return ""

            output = []
            for param_name, param_value in params.items():
                output.append(f"{param_name}: {str(param_value)}")
            return " (" + ", ".join(output) + ")"

        # Sanity check
        if stack_element is not None:
            assert (
                stack_element["type"] == tree_element["type"]
            ), f"The stack and the tree do not match (element type mismatch: {stack_element['type']} vs. {tree_element['type']})"
            if stack_element["type"] != "sequence":
                assert (
                    stack_element["name"] == tree_element["name"]
                ), f"The stack and the tree do not match (element name mismatch: {stack_element['name']} vs. {tree_element['name']})"

        # Initialize parameters of the dot node we are going to create
        dot_node_params = {
            "name": str(uuid.uuid4()),
        }

        # Go through all possible element types and create the corresponding label and shape
        if tree_element["type"] == "sequence":
            dot_node_params["shape"] = "box"

            # If we have a sequence we have to create a label for each action and mark the current one (if one is on the stack)
            label = ["Sequence:"]
            for i, action in enumerate(tree_element["action_elements"]):
                # Spaces for indentation
                action_label = "  "
                # Mark current element (if this sequence is on the stack)
                if stack_element is not None and i == stack_element["current_action_index"]:
                    action_label += "--> "
                action_label += action["name"] + param_string(action["parameters"])
                label.append(action_label)
            dot_node_params["label"] = "\n".join(label)
        elif tree_element["type"] == "decision":
            dot_node_params["shape"] = "ellipse"
            dot_node_params["label"] = tree_element["name"] + param_string(tree_element["parameters"])
        elif tree_element["type"] == "action":
            dot_node_params["shape"] = "box"
            dot_node_params["label"] = tree_element["name"] + param_string(tree_element["parameters"])
        else:
            raise ParseError(f"Unknown element type {tree_element['type']}")

        # Set color if this element is on the stack
        dot_node_params["color"] = "gray" if stack_element is None else "orangered"

        # Create node in graph
        return pydot.Node(**dot_node_params)

    def _stack_to_dotgraph(
        self, dot: pydot.Dot, subtree_root: dict, stack_root: Optional[dict] = None, full_tree: bool = False
    ) -> (pydot.Dot, str):
        """
        Recursively modify dot to include every element of the stack

        :param dot: The dotgraph to modify
        :param subtree_root: The root node of the subtree which should be added to the graph
        :param stack_root: The root node / bottom of the stack which should be added to the graph (this corresponds to subtree_root as they are different representations of the same component). The stack is optional as certain subtrees might not be in contact with the stack.
        :param full_tree: If true the whole tree is rendered, otherwise only the current stack and its direct neighbors are rendered
        :return: The modified dotgraph and the uid of the root node
        """
        # Sanity check
        if stack_root is not None:
            assert (
                stack_root["type"] == subtree_root["type"]
            ), f"The stack element type and the tree element type do not match ({stack_root['type']} vs. {subtree_root['type']})"
            if stack_root["type"] != "sequence":
                assert (
                    stack_root["name"] == subtree_root["name"]
                ), f"The stack element name and the tree element name do not match ({stack_root['name']} vs. {subtree_root['name']})"

        # Generate dot node for the root and mark it as active if it is on the stack
        dot_node = DsdFollower._dot_node_from_tree_element(subtree_root, stack_root)
        # Append this element to graph
        dot.add_node(dot_node)

        # Append children to graph if this element is on the stack or if we want to render the whole tree
        if "children" in subtree_root and (stack_root is not None or full_tree):
            # Go through all children
            for activating_result, child in subtree_root["children"].items():
                # Get the root of the sub stack if this branch of the tree is the one which is currently on the stack
                sub_stack_root = None
                if (
                    stack_root is not None
                    and stack_root["next"] is not None
                    and stack_root["next"]["activation_reason"] == activating_result
                ):
                    sub_stack_root = stack_root["next"]

                # Recursively generate dot nodes for the children
                dot, child_uid = self._stack_to_dotgraph(dot, child, sub_stack_root, full_tree)
                # Connect the child to the parent element
                edge = pydot.Edge(src=dot_node.get_name(), dst=child_uid, label=activating_result)
                dot.add_edge(edge)
        return dot, dot_node.get_name()

    def _append_debug_data_to_item(
        self, parent_item: QStandardItem, debug_data: Union[dict, list, int, float, str, bool]
    ):
        """
        Appends debug_data of a given element and its children to a QStandardItem.
        """
        if isinstance(debug_data, list):
            for i, data in enumerate(debug_data):
                child_item = QStandardItem()
                child_item.setText(str(i) + ": ")
                child_item.setEditable(False)
                child_item.setSelectable(False)
                self._append_debug_data_to_item(child_item, data)
                parent_item.appendRow(child_item)
        elif isinstance(debug_data, dict):
            for label, data in debug_data.items():
                child_item = QStandardItem()
                child_item.setText(str(label) + ": ")
                child_item.setEditable(False)
                child_item.setSelectable(False)
                self._append_debug_data_to_item(child_item, data)
                parent_item.appendRow(child_item)
        elif isinstance(debug_data, (bool, float, int, str, bytes)):
            parent_item.setText(parent_item.text() + str(debug_data))

    def to_dotgraph(self, full_tree: bool) -> pydot.Dot:
        """
        Represent the current stack as dotgraph

        :param full_tree: If true the whole tree is rendered, otherwise only the current stack is rendered
        """
        # Return cached result if available
        if self._cached_dotgraph is not None:
            return self._cached_dotgraph

        self._node.get_logger().debug("Generating dotgraph")

        # Check if we received any data yet
        if self.stack is None or self.tree is None:
            self._node.get_logger().info("No data received yet")
            return self._error_dotgraph()

        # Create dot graph
        self._cached_dotgraph, _ = self._stack_to_dotgraph(
            pydot.Dot(graph_type="digraph"), self.tree, self.stack, full_tree
        )

        return self._cached_dotgraph

    def to_q_item_model(self):
        """
        Represent the DSDs debug data as QITemModel
        """
        # Return if we received no data yet
        if self.stack is None or self.tree is None:
            return self._empty_item_model()

        # Construct a new item-model
        model = QStandardItemModel()

        # Start with the root/bottom of the stack
        stack_element = self.stack

        # Store the corresponding tree element
        tree_element = self.tree

        # Go through all stack elements
        while stack_element is not None:
            # Sanity check
            assert "next" in stack_element, "Stack element has no next element"
            # Check if this is the last element
            last_element: bool = stack_element["next"] is None

            # Create a new item for this element
            elem_item = QStandardItem()
            elem_item.setEditable(False)
            elem_item.setSelectable(False)

            # Set the text of the item
            if stack_element["type"] == "sequence":
                # Get the names of all actions
                action_names = [action["name"] for action in tree_element["action_elements"]]
                # Join them together and set the text
                elem_item.setText("Sequence: " + ", ".join(action_names))
            else:
                elem_item.setText(stack_element["name"])

            # Add debug data to the item
            self._append_debug_data_to_item(elem_item, stack_element["debug_data"])

            model.invisibleRootItem().appendRow(elem_item)

            # Add a spacer
            if not last_element:
                # Create a new horizontal line covering the full with as a spacer of the stack elements
                spacer = QStandardItem()
                spacer.setEditable(False)
                # Disable the selection of the spacer
                spacer.setSelectable(False)
                spacer.setText("──────────────────────────────")

                model.invisibleRootItem().appendRow(spacer)

            # Go to next element
            stack_element = stack_element["next"]
            # Also select the corresponding tree element if there are any
            if "children" in tree_element and stack_element is not None:
                tree_element = tree_element["children"][stack_element["activation_reason"]]

        return model

    def destroy(self):
        """
        Cleanup the subscriptions, so we don't receive any more data that we don't need.
        This improves the performance.
        Also this allows the garbage collector to delete this object and prevent memory leaks.
        """
        self._node.destroy_subscription(self.tree_sub)
        self._node.destroy_subscription(self.stack_sub)

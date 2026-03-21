# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys
import uuid
from typing import Optional

import pydot
from ament_index_python import get_package_share_directory
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIcon, QPainter, QStandardItemModel
from PyQt5.QtSvg import QSvgGenerator
from PyQt5.QtWidgets import QFileDialog, QGraphicsScene, QWidget
from PyQt5.uic import loadUi
from qt_dotgraph.dot_to_qt import DotToQtGenerator
from qt_dotgraph.pydotfactory import PydotFactory
from rclpy.node import Node
from rqt_gui.main import Main
from rqt_gui_py.plugin import Plugin

from .dsd_follower import DsdFollower
from .interactive_graphics_view import InteractiveGraphicsView


class DsdVizPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self._node: Node = context.node
        self._initialized = False  # This gets set to true once the plugin hast completely finished loading

        # Ensure startup state
        self.freeze = False  # Controls whether the state should be updated from remote
        self.dsd_follower: Optional[DsdFollower] = None
        self.running_dsd_instances: dict[str, str] = {}
        self._init_plugin(context)

        # Performance optimization variables
        self._prev_dotgraph = None
        self._prev_QItemModel = None

    def _init_plugin(self, context):
        self.setObjectName("DSD-Visualization")

        self._widget = QWidget()
        self._widget.setObjectName(self.objectName())

        # load qt ui definition from file
        ui_file = os.path.join(
            get_package_share_directory("dynamic_stack_decider_visualization"), "resource", "StackmachineViz.ui"
        )
        loadUi(ui_file, self._widget, {"InteractiveGraphicsView": InteractiveGraphicsView})

        # initialize qt scene
        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)
        self._widget.graphics_view.setScene(self._scene)

        # Bind fit-in-view button
        self._widget.fit_in_view_push_button.setIcon(QIcon.fromTheme("view-fullscreen"))
        self._widget.fit_in_view_push_button.pressed.connect(self.fit_in_view)

        # Fit-in-view on checkbox toggle
        self._widget.auto_fit_graph_check_box.toggled.connect(self.fit_in_view)

        # Freezing
        def toggle_freeze():
            self.freeze = not self.freeze
            self.refresh()

        self._widget.freeze_push_button.toggled.connect(toggle_freeze)

        # Exporting and importing
        self._widget.save_as_svg_push_button.pressed.connect(self.save_svg_to_file)

        # Discover dsd instances
        self.discover_dsd_instances()

        # Connect the combo box to the set_dsd method
        self._widget.dsd_selector_combo_box.currentTextChanged.connect(self.set_dsd)

        # Bind refresh button and add icon
        self._widget.refresh_combobox_push_button.setIcon(QIcon.fromTheme("view-refresh"))
        self._widget.refresh_combobox_push_button.pressed.connect(self.discover_dsd_instances)

        # Hook that resets the dot cache and thus forces a redraw of the dotgraph
        def reset_dot_cache():
            if self.dsd_follower is not None:
                self.dsd_follower.reset_cache()

        # Add hook that triggers redraw to ui elements
        self._widget.highlight_connections_check_box.toggled.connect(reset_dot_cache)
        self._widget.show_full_tree.toggled.connect(reset_dot_cache)
        self._widget.auto_fit_graph_check_box.toggled.connect(reset_dot_cache)
        self._widget.fit_in_view_push_button.pressed.connect(reset_dot_cache)
        self._widget.freeze_push_button.toggled.connect(reset_dot_cache)

        # Add to InteractiveGraphicsView mouse wheel event that disables auto-fit-in-view
        def wheel_event(event):
            self._widget.auto_fit_graph_check_box.setChecked(False)
            self._widget.graphics_view.__class__.wheelEvent(self._widget.graphics_view, event)

        # Overwrite the original wheelEvent handler
        self._widget.graphics_view.wheelEvent = wheel_event

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Start a timer that calls back every 100 ms
        self._timer_id = self.startTimer(100)

    def discover_dsd_instances(self):
        """Updates the dict of known dsd instances"""
        # Store all DSDs in a dict
        dsd_instances = {}

        # List all known topics
        topics = self._node.get_topic_names_and_types()
        for topic_name, topic_types in topics:
            if topic_name.endswith("/dsd_tree") and "std_msgs/msg/String" in topic_types:
                # Extract the dsd name from the topic name
                dsd_name = "/".join(topic_name.split("/")[:-1]).upper()
                # Store the dsd name and the debug topic namespace (not just the tree topic)
                dsd_instances[dsd_name] = topic_name.replace("/dsd_tree", "")

        # Update the list of known dsd instances
        self.running_dsd_instances = dsd_instances

        # Update the list of dsd instances in the combo box
        self._widget.dsd_selector_combo_box.clear()
        self._widget.dsd_selector_combo_box.addItem("Select DSD...")
        for name in self.running_dsd_instances.keys():
            self._widget.dsd_selector_combo_box.addItem(name)

    def save_settings(self, plugin_settings, instance_settings):
        super().save_settings(plugin_settings, instance_settings)

        instance_settings.set_value("auto_fit_graph_check_box_state", self._widget.auto_fit_graph_check_box.isChecked())
        instance_settings.set_value(
            "highlight_connections_check_box_state", self._widget.highlight_connections_check_box.isChecked()
        )
        instance_settings.set_value("show_full_tree_check_box_state", self._widget.show_full_tree.isChecked())

    def restore_settings(self, plugin_settings, instance_settings):
        super().restore_settings(plugin_settings, instance_settings)

        self._widget.auto_fit_graph_check_box.setChecked(
            instance_settings.value("auto_fit_graph_check_box_state", True) in [True, "true"]
        )
        self._widget.highlight_connections_check_box.setChecked(
            instance_settings.value("highlight_connections_check_box_state", True) in [True, "true"]
        )
        self._widget.show_full_tree.setChecked(
            instance_settings.value("show_full_tree_check_box_state", False) in [True, "true"]
        )

        self._initialized = True
        self.refresh()

    def save_svg_to_file(self):
        file_name, _ = QFileDialog.getSaveFileName(
            self._widget, self.tr("Save as SVG"), "stackmachine.svg", self.tr("Scalable Vector Graphic (*.svg)")
        )

        if file_name is not None and file_name != "":
            generator = QSvgGenerator()
            generator.setFileName(file_name)
            generator.setSize((self._scene.sceneRect().size() * 2.0).toSize())

            painter = QPainter(generator)
            painter.setRenderHint(QPainter.Antialiasing)
            self._scene.render(painter)
            painter.end()

    # fmt: off
    def timerEvent(self, timer_event): #noqa: N802
        # fmt: on
        """This gets called by QT whenever the timer ticks"""
        # Refresh the viz if the freeze button is not pressed
        if not self.freeze:
            self.refresh()

        # Automatically fit in view if the checkbox is checked
        if self._widget.auto_fit_graph_check_box.isChecked():
            self.fit_in_view()

    def fit_in_view(self):
        """Rescales the tree to fit the window"""
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

    def refresh(self):
        """Refresh the complete drawn representation"""

        # Show message if the plugin is not yet completely initialized
        if not self._initialized:
            self._render_messages("The plugin is not yet completely initialized. Please wait...")

        # Show message if no dsd is selected
        elif self.dsd_follower is None:
            self._render_messages("No DSD selected")

        else:
            # Render the dotgraph and the debug data
            self._render_dotgraph(self.dsd_follower.to_dotgraph(
                full_tree=self._widget.show_full_tree.isChecked(),
            ))
            self._render_debug_data(self.dsd_follower.to_q_item_model())

    def _render_messages(self, *messages):
        """Render simple messages on the canvas"""

        msg_dot = pydot.Dot()

        for msg in messages:
            msg_dot.add_node(pydot.Node(str(uuid.uuid4()), label=str(msg)))

        self._render_dotgraph(msg_dot)
        self._render_debug_data(QStandardItemModel(self._scene))

    def _render_dotgraph(self, dotgraph):
        """Render the specified dotgraph on canvas"""

        # Only redraw when the graph differs from the previous one
        if self._prev_dotgraph == dotgraph:
            return
        else:
            self._prev_dotgraph = dotgraph

        self._scene.clear()

        if self._widget.highlight_connections_check_box.isChecked():
            highlight_level = 3
        else:
            highlight_level = 1

        # Generate qt items from dotcode
        dotcode = PydotFactory().create_dot(dotgraph)
        nodes, edges = DotToQtGenerator().dotcode_to_qt_items(dotcode, highlight_level, same_label_siblings=False)

        # Add generated items to scene
        for node_item in nodes:
            self._scene.addItem(nodes.get(node_item))
        for edge_items in edges:
            for edge_item in edges.get(edge_items):
                edge_item.add_to_scene(self._scene)

        self._scene.setSceneRect(self._scene.itemsBoundingRect())

    def _render_debug_data(self, qitem_model):
        """Render debug data in the tree view on the right side of the scene"""

        # Only redraw when the item-model differs from the previous one
        if self._prev_QItemModel == qitem_model:
            return
        else:
            self._prev_QItemModel = qitem_model
            self._widget.stack_prop_tree_view.setHeaderHidden(True)
            self._widget.stack_prop_tree_view.setModel(qitem_model)
            self._widget.stack_prop_tree_view.expandAll()

    def set_dsd(self, name):
        """
        Set the target dsd

        :param name: display_name of any dsd in the locations.yaml
        """
        if name == "Select DSD...":
            self.dsd_follower = None
            return

        # Check if the selected dsd is known
        if name not in self.running_dsd_instances.keys():
            return

        # Destroy the old dsd follower, so that we don't keep old subscriptions that take up resources
        if self.dsd_follower is not None:
            self.dsd_follower.destroy()

        # Initialize dsd follower
        self.dsd_follower = DsdFollower(self._node, self.running_dsd_instances[name])


def main():
    plugin = "dynamic_stack_decider_visualization.dsd_visualization_plugin.DsdVizPlugin"
    main = Main(filename=plugin)
    sys.exit(main.main(standalone=plugin))


if __name__ == "__main__":
    main()

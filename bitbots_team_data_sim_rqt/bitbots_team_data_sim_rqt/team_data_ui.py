#!/usr/bin/env python3
import math
import os
import sys

import rclpy
from ament_index_python import get_package_share_directory
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QFileDialog,
    QGroupBox,
    QLabel,
    QLineEdit,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QShortcut,
    QTreeWidgetItem,
    QVBoxLayout,
)
from PyQt5.uic import loadUi
from rclpy.node import Node
from rqt_gui.main import Main
from rqt_gui_py.plugin import Plugin

from bitbots_msgs.msg import TeamData


class TeamDataSimulator(Plugin):
    """
    This class is the main class for the RecordUI. It is a plugin for the rqt framework and is used to record animations.
    """

    def __init__(self, context):
        super().__init__(context)
        # Store reference to node
        self._node: Node = context.node

        # Set Name of the plugin
        self.setObjectName("Team Data Simulator")

        # Create publishers
        self.team_data_pub = self._node.create_publisher(TeamData, "/team_data", 1)

        # Initialize the window
        self._widget = QMainWindow()

        # Load XML ui definition
        ui_file = os.path.join(
            get_package_share_directory("bitbots_team_data_sim_rqt"), "resource", "RobotTeamDataSimulator.ui"
        )
        self._widget: QMainWindow = loadUi(ui_file)

        # Connect callbacks to GUI components
        self.connect_gui_callbacks()

        # Add the widget to the context
        context.add_widget(self._widget)

        # Tell the user that the initialization is complete
        self._node.get_logger().info("Initialization complete.")




    def connect_gui_callbacks(self) -> None:
        """
        Connects the actions in the top bar to the corresponding functions, and sets their shortcuts
        :return:
        """
        return
        self._widget.actionNew.triggered.connect(self.new)
        self._widget.actionOpen.triggered.connect(self.open)
        self._widget.actionSave.triggered.connect(self.save)
        self._widget.actionSave_as.triggered.connect(lambda: self.save(new_location=True))
        self._widget.actionInit.triggered.connect(self.goto_init)
        self._widget.actionCurrent_Frame.triggered.connect(self.goto_frame)
        self._widget.actionNext_Frame.triggered.connect(self.goto_next)
        self._widget.actionAnimation.triggered.connect(self.play)
        self._widget.actionAnimation_until_Frame.triggered.connect(self.play_until)
        self._widget.actionDuplicate_Frame.triggered.connect(self.duplicate)
        self._widget.actionDelete_Frame.triggered.connect(self.delete)
        self._widget.actionLeft.triggered.connect(lambda: self.mirror_frame("L"))
        self._widget.actionRight.triggered.connect(lambda: self.mirror_frame("R"))
        self._widget.actionInvert.triggered.connect(self.invert_frame)
        self._widget.actionUndo.triggered.connect(self.undo)
        self._widget.actionRedo.triggered.connect(self.redo)
        self._widget.actionWalkready.triggered.connect(self.play_walkready)
        self._widget.actionHelp.triggered.connect(self.help)
        self._widget.buttonRecord.clicked.connect(self.record)
        self._widget.buttonRecord.shortcut = QShortcut(QKeySequence("Space"), self._widget)
        self._widget.buttonRecord.shortcut.activated.connect(self.record)
        self._widget.frameList.key_pressed.connect(self.delete)
        self._widget.frameList.itemSelectionChanged.connect(self.frame_select)

def main():
    plugin = "bitbots_team_data_sim_rqt.team_data_ui.TeamDataSimulator"
    main = Main(filename=plugin)
    sys.exit(main.main(standalone=plugin))

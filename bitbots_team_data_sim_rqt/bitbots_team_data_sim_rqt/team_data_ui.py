#!/usr/bin/env python3
import os
import sys

from ament_index_python import get_package_share_directory
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QComboBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QRadioButton,
    QShortcut,
    QSlider,
    QSpinBox,
    QVBoxLayout,
)
from PyQt5.uic import loadUi
from rclpy.node import Node
from rqt_gui.main import Main
from rqt_gui_py.plugin import Plugin

from bitbots_msgs.msg import TeamData


class RobotWidget(QGroupBox):
    def __init__(self, parent=None):
        super(RobotWidget, self).__init__(parent)
        self.setTitle("Robot")
        # Set maximum width
        self.setMaximumWidth(350)
        self.setMinimumWidth(150)
        # Create layout
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)
        self.id_layout = QFormLayout()
        self.main_layout.addLayout(self.id_layout)
        self.id_spin_box = QSpinBox()
        self.id_spin_box.setRange(0, 4)
        self.id_layout.addRow(QLabel("ID"), self.id_spin_box)
        self.state_box = StateBox()
        self.main_layout.addWidget(self.state_box)
        self.time_to_position_box = TimeToPositionBox()
        self.main_layout.addWidget(self.time_to_position_box)
        self.strategy_box = StrategyBox()
        self.main_layout.addWidget(self.strategy_box)
        self.publish_button = PublishButton()
        self.main_layout.addWidget(self.publish_button)

        self.id_spin_box.valueChanged.connect(self.id_spin_box_update)

    def id_spin_box_update(self):
        enabled = self.id_spin_box.value() != 0
        self.state_box.setEnabled(enabled)
        self.time_to_position_box.setEnabled(enabled)
        self.strategy_box.setEnabled(enabled)
        self.publish_button.setEnabled(enabled)


class StateBox(QGroupBox):
    def __init__(self, parent=None):
        super(StateBox, self).__init__(parent)
        self.setTitle("State")
        self.setEnabled(False)
        # Create layout
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)
        self.unknown_button = QRadioButton("Unknown")
        self.unknown_button.setChecked(True)
        self.main_layout.addWidget(self.unknown_button)
        self.penalized_button = QRadioButton("Penalized")
        self.main_layout.addWidget(self.penalized_button)
        self.unpenalized_button = QRadioButton("Unpenalized")
        self.main_layout.addWidget(self.unpenalized_button)

    def get_state(self):
        if self.unknown_button.isChecked():
            return 0
        if self.penalized_button.isChecked():
            return 1
        if self.unpenalized_button.isChecked():
            return 2
        return 0


class TimeToPositionBox(QGroupBox):
    def __init__(self, parent=None):
        super(TimeToPositionBox, self).__init__(parent)
        self.setTitle("Time to Position at Ball")
        self.setEnabled(False)
        # Create layout
        self.main_layout = QHBoxLayout()
        self.setLayout(self.main_layout)
        self.time_spin_box = QSpinBox()
        self.time_spin_box.setRange(0, 200)
        self.main_layout.addWidget(self.time_spin_box)
        self.time_slider = QSlider(Qt.Horizontal)
        self.time_slider.setRange(0, 200)
        self.main_layout.addWidget(self.time_slider)
        self.time_spin_box.valueChanged.connect(self.time_slider.setValue)
        self.time_slider.valueChanged.connect(self.time_spin_box.setValue)


class StrategyBox(QGroupBox):
    def __init__(self, parent=None):
        super(StrategyBox, self).__init__(parent)
        self.setTitle("Strategy")
        self.setEnabled(False)
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)
        self.combobox = QComboBox()
        self.combobox.addItem("Striker")
        self.combobox.addItem("Offense")
        self.combobox.addItem("Supporter")
        self.combobox.addItem("Defender")
        self.combobox.addItem("Defense")
        self.combobox.addItem("Other")
        self.combobox.addItem("Goalie")
        self.combobox.addItem("Idle")
        self.main_layout.addWidget(self.combobox)


class PublishButton(QPushButton):
    def __init__(self, parent=None):
        super(PublishButton, self).__init__(parent)
        self.button_text = "Publish Team Data"
        self.setText(self.button_text)
        self.setCheckable(True)
        self.setEnabled(False)

        self.clicked.connect(self.publish_button_clicked)

    def publish_button_clicked(self):
        if self.isChecked():
            self.setText("Publishing Team Data")
        else:
            self.setText(self.button_text)


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

        # Add widgets to the window
        robot_widget = RobotWidget()
        self._widget.robots_layout.addWidget(robot_widget)
        self.robots = [robot_widget]

        # Connect callbacks to GUI components
        self.connect_gui_callbacks()

        # Add the widget to the context
        context.add_widget(self._widget)

        # Tell the user that the initialization is complete
        self._node.get_logger().info("Initialization complete.")

    def plus_button_clicked(self):
        robot_widget = RobotWidget()
        self._widget.robots_layout.addWidget(robot_widget)
        self.robots.append(robot_widget)
        self._widget.MinusButton.setEnabled(True)
        if len(self.robots) >= 4:
            self._widget.PlusButton.setEnabled(False)

    def minus_button_clicked(self):
        if len(self.robots) > 1:
            robot_widget = self.robots.pop()
            robot_widget.deleteLater()
            if len(self.robots) < 4:
                self._widget.PlusButton.setEnabled(True)
        if len(self.robots) == 1:
            self._widget.MinusButton.setEnabled(False)

    def connect_gui_callbacks(self) -> None:
        """
        Connects the actions in the top bar to the corresponding functions, and sets their shortcuts
        :return:
        """
        # Show test message box
        self._widget.PlusButton.clicked.connect(self.plus_button_clicked)
        self._widget.MinusButton.clicked.connect(self.minus_button_clicked)
        

def main():
    plugin = "bitbots_team_data_sim_rqt.team_data_ui.TeamDataSimulator"
    main = Main(filename=plugin)
    sys.exit(main.main(standalone=plugin))

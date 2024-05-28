#!/usr/bin/env python3
import os
import sys

from ament_index_python import get_package_share_directory
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QComboBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QRadioButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
)
from PyQt5.uic import loadUi
from rclpy.node import Node
from rqt_gui.main import Main
from rqt_gui_py.plugin import Plugin

from bitbots_msgs.msg import Strategy, TeamData


class RobotWidget(QGroupBox):
    def __init__(self, parent=None):
        super().__init__(parent)
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

    def get_robot_id(self) -> int:
        return self.id_spin_box.value()

    def active(self) -> bool:
        return self.publish_button.isChecked()


class StateBox(QGroupBox):
    _states: dict[int, str] = {
        TeamData.STATE_UNKNOWN: "Unknown",
        TeamData.STATE_PENALIZED: "Penalized",
        TeamData.STATE_UNPENALIZED: "Unpenalized",
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("State")
        self.setEnabled(False)
        # Create layout
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)
        # Create radio buttons
        self.radio_buttons: dict[int, QRadioButton] = {}
        for state_id, name in self._states.items():
            button = QRadioButton(name)
            self.main_layout.addWidget(button)
            self.radio_buttons[state_id] = button
        # Set default state
        self.default_state = TeamData.STATE_UNKNOWN
        self.radio_buttons[self.default_state].setChecked(True)

    def get_state(self) -> int:
        for state_id, button in self.radio_buttons.items():
            if button.isChecked():
                return state_id
        return self.default_state


class TimeToPositionBox(QGroupBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Time to Position at Ball")
        self.setEnabled(False)
        # Create layout
        self.main_layout = QHBoxLayout()
        self.setLayout(self.main_layout)
        # Create spin box
        self.time_spin_box = QSpinBox()
        self.time_spin_box.setRange(0, 200)
        self.main_layout.addWidget(self.time_spin_box)
        # Create slider
        self.time_slider = QSlider(Qt.Horizontal)
        self.time_slider.setRange(0, 200)
        self.main_layout.addWidget(self.time_slider)
        # Connect spin box and slider
        self.time_spin_box.valueChanged.connect(self.time_slider.setValue)
        self.time_slider.valueChanged.connect(self.time_spin_box.setValue)

    def get_time(self) -> float:
        return float(self.time_spin_box.value())


class StrategyBox(QGroupBox):
    _roles: dict[str, int] = {
        "Undefined": Strategy.ROLE_UNDEFINED,
        "Defender": Strategy.ROLE_DEFENDER,
        "Goalie": Strategy.ROLE_GOALIE,
        "Idling": Strategy.ROLE_IDLING,
        "Striker": Strategy.ROLE_STRIKER,
        "Supporter": Strategy.ROLE_SUPPORTER,
        "Other": Strategy.ROLE_OTHER,
    }
    _actions: dict[str, int] = {
        "Undefined": Strategy.ACTION_UNDEFINED,
        "Going to Ball": Strategy.ACTION_GOING_TO_BALL,
        "Kicking": Strategy.ACTION_KICKING,
        "Localizing": Strategy.ACTION_LOCALIZING,
        "Positioning": Strategy.ACTION_POSITIONING,
        "Searching": Strategy.ACTION_SEARCHING,
        "Trying to Score": Strategy.ACTION_TRYING_TO_SCORE,
        "Waiting": Strategy.ACTION_WAITING,
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Strategy")
        self.setEnabled(False)

        # Create layout
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        # Create combobox for role
        self.role_combobox = QComboBox()
        for action in self._roles.keys():
            self.role_combobox.addItem(action)
        self.main_layout.addWidget(self.role_combobox)

        # Create combobox for action
        self.action_combobox = QComboBox()
        for action in self._actions.keys():
            self.action_combobox.addItem(action)
        self.main_layout.addWidget(self.action_combobox)

    def get_role(self) -> int:
        return self._roles[self.role_combobox.currentText()]

    def get_action(self) -> int:
        return self._actions[self.action_combobox.currentText()]


class PublishButton(QPushButton):
    def __init__(self, parent=None):
        super().__init__(parent)
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


class TeamDataSimulator(Plugin):  # TODO add sim time button
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

        self.timer = self._node.create_timer(0.1, self.timer_callback)

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

    def timer_callback(self):
        for robot_widget in self.robots:
            if robot_widget.active():
                team_data = TeamData()
                team_data.robot_id = robot_widget.get_robot_id()
                team_data.state = robot_widget.state_box.get_state()
                team_data.time_to_position_at_ball = robot_widget.time_to_position_box.get_time()
                team_data.strategy.role = robot_widget.strategy_box.get_role()
                team_data.strategy.action = robot_widget.strategy_box.get_action()
                team_data.header.stamp = self._node.get_clock().now().to_msg()
                self.team_data_pub.publish(team_data)

    def connect_gui_callbacks(self) -> None:
        """
        Connects the actions in the top bar to the corresponding functions, and sets their shortcuts
        :return:
        """
        # Show test message box
        self._widget.PlusButton.clicked.connect(self.plus_button_clicked)
        self._widget.MinusButton.clicked.connect(self.minus_button_clicked)

    def shutdown_plugin(self):
        """Clean up on shutdown"""
        self.timer.cancel()


def main():
    plugin = "bitbots_team_data_sim_rqt.team_data_ui.TeamDataSimulator"
    main = Main(filename=plugin)
    sys.exit(main.main(standalone=plugin))

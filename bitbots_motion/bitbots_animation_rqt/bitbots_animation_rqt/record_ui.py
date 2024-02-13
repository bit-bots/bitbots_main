#!/usr/bin/env python3
import math
import os
import sys

from ament_index_python import get_package_share_directory
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import (
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
from rclpy.action import ActionClient
from rclpy.node import Node
from rqt_gui.main import Main
from rqt_gui_py.plugin import Plugin
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from bitbots_animation_rqt.animation_recording import Recorder
from bitbots_animation_rqt.utils import DragDropList
from bitbots_msgs.action import PlayAnimation
from bitbots_msgs.msg import JointCommand, JointTorque
from bitbots_msgs.srv import AddAnimation


class RecordUI(Plugin):
    """
    This class is the main class for the RecordUI. It is a plugin for the rqt framework and is used to record animations.
    """

    def __init__(self, context):
        super().__init__(context)
        # Store reference to node
        self._node: Node = context.node

        # Set Name of the plugin
        self.setObjectName("Record Animation")

        # Create publishers
        self._joint_pub = self._node.create_publisher(JointCommand, "record_motor_goals", 1)
        self.effort_pub = self._node.create_publisher(JointTorque, "ros_control/set_torque_individual", 1)

        # Create a action client to play animations
        self.animation_client: ActionClient = ActionClient(self._node, PlayAnimation, "animation")

        # Create a service clients
        self.add_animation_client = self._node.create_client(AddAnimation, "add_temporary_animation")
        self.hcm_record_mode_client = self._node.create_client(SetBool, "record_mode")

        # Initialize the recorder module
        self.create_initialized_recorder()

        # Initialize the window
        self._widget = QMainWindow()

        # Load XML ui definition
        ui_file = os.path.join(get_package_share_directory("bitbots_animation_rqt"), "resource", "RecordUI.ui")
        loadUi(ui_file, self._widget, {})

        # Initialize the GUI state
        self._motor_controller_text_fields = {}
        self._joint_states = JointState()

        # Initialize the working values
        self._working_angles: dict[str, float] = {}

        self.update_time = 0.1  # TODO what is this for?

        # QT directory for saving files
        self._save_directory = None

        # Initialize the motor tree structure where we can select which motors are stiff
        self._motor_switcher_active_checkbox: dict[str, QTreeWidgetItem] = {}
        self._motor_switcher_torque_checkbox: dict[str, QTreeWidgetItem] = {}

        # Motor hierarchy
        self._motor_hierarchy = {  # TODO this should be a parameter / loaded from the urdf
            "Body": {
                "Head": ["HeadPan", "HeadTilt"],
                "Arms": {
                    "Left": [
                        "LShoulderPitch",
                        "LShoulderRoll",
                        "LElbow",
                    ],
                    "Right": [
                        "RShoulderPitch",
                        "RShoulderRoll",
                        "RElbow",
                    ],
                },
                "Legs": {
                    "Left": [
                        "LHipYaw",
                        "LHipRoll",
                        "LHipPitch",
                        "LKnee",
                        "LAnklePitch",
                        "LAnkleRoll",
                    ],
                    "Right": [
                        "RHipYaw",
                        "RHipRoll",
                        "RHipPitch",
                        "RKnee",
                        "RAnklePitch",
                        "RAnkleRoll",
                    ],
                },
            }
        }

        # Create drag and dop list for keyframes
        self._widget.frameList = DragDropList(self._widget, self)
        self._widget.verticalLayout_2.insertWidget(0, self._widget.frameList)
        self._widget.frameList.setDragDropMode(QAbstractItemView.InternalMove)

        # Create a list of all motors
        def get_motor_names(hierarchy: dict) -> list:
            names = []
            for element in hierarchy.values():
                if isinstance(element, dict):
                    names.extend(get_motor_names(element))
                else:
                    names.extend(element)
            return names

        self.motors = get_motor_names(self._motor_hierarchy)

        # Create the initial joint state
        self._initial_joints = JointState(
            name=self.motors,
            position=[0.0] * len(self.motors),
        )

        # Create GUI components
        self.create_motor_controller()
        self.create_motor_switcher()
        # Update ticks
        self.react_to_motor_selection()
        self.update_frames()
        # Connect callbacks to GUI components
        self.connect_gui_callbacks()

        # Add the widget to the context
        context.add_widget(self._widget)

        # Create subscriptions
        self.state_sub = self._node.create_subscription(JointState, "joint_states", self.joint_state_callback, 1)

        # Tell the user that the initialization is complete
        self._node.get_logger().info("Initialization complete.")
        self._widget.statusBar.showMessage("Initialization complete.")

    def joint_state_callback(self, joint_states: JointState) -> None:
        """
        Callback method for /joint_states.
        """
        # Store the joint states
        self._joint_states = joint_states

    def create_motor_controller(self) -> None:
        """
        Sets up the GUI in the middle of the Screen to control the motors.
        Uses self._motorValues to determine which motors are present.
        """
        # Iterate over all motors
        for i, motor_name in enumerate(sorted(self.motors)):
            # Create a group of UI elements for each motor
            group = QGroupBox()
            layout = QVBoxLayout()
            # Create a label for the motor name
            label = QLabel()
            label.setText(motor_name)
            layout.addWidget(label)

            # Add a textfield to display the exact value of the motor
            textfield = QLineEdit()
            textfield.setText("0.0")
            textfield.textEdited.connect(self.textfield_update)
            layout.addWidget(textfield)
            self._motor_controller_text_fields[motor_name] = textfield

            # Add the layout to the group and the group to the main layout (at the correct position in the 5 x n grid)
            group.setLayout(layout)
            self._widget.motorControlLayout.addWidget(group, i // 5, i % 5)

    def create_motor_switcher(self) -> None:
        """
        Loads the motors into the tree and adds the checkboxes
        """

        # Create a recursive function to build the tree
        def build_widget_tree(parent, hierarchy: dict) -> None:
            # Iterate over all elements in the hirarchy
            for key, value in hierarchy.items():
                # If the element is a dict, create a new group in the tree
                if isinstance(value, dict):
                    # Create a new group in the tree
                    child = QTreeWidgetItem(parent)
                    child.setText(0, key)
                    child.setFlags(child.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
                    child.setExpanded(True)
                    # Recursively call the function to add the children of the group
                    build_widget_tree(child, value)
                # If the element is a list we are at the lowest level of the hirarchy and add the motors
                elif isinstance(value, list):
                    # Create a new group in the tree
                    child = QTreeWidgetItem(parent)
                    child.setText(0, key)
                    child.setFlags(child.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
                    child.setExpanded(True)
                    for motor_name in value:
                        # Add motor enable checkbox
                        enable_checkbox = QTreeWidgetItem(child)
                        enable_checkbox.setText(0, motor_name)
                        enable_checkbox.setFlags(enable_checkbox.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
                        enable_checkbox.setCheckState(0, Qt.Checked)
                        enable_checkbox.setExpanded(True)
                        self._motor_switcher_active_checkbox[motor_name] = enable_checkbox

                        # Put a torque checkbox below the motor enable checkbox
                        torque_checkbox = QTreeWidgetItem(enable_checkbox)
                        torque_checkbox.setText(0, "Torque")
                        torque_checkbox.setFlags(torque_checkbox.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
                        torque_checkbox.setCheckState(0, Qt.Checked)
                        self._motor_switcher_torque_checkbox[motor_name] = torque_checkbox
                else:
                    raise ValueError("Invalid hirarchy")

        # Build the tree
        build_widget_tree(self._widget.motorTree, self._motor_hierarchy)
        self._widget.motorTree.setHeaderLabel("Active / Stiff Motors")

        # Register hook that executes our callback when the user clicks on a checkbox
        self._widget.motorTree.itemClicked.connect(self.react_to_motor_selection)

    def connect_gui_callbacks(self) -> None:
        """
        Connects the actions in the top bar to the corresponding functions, and sets their shortcuts
        :return:
        """
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
        self._widget.actionHelp.triggered.connect(self.help)
        self._widget.buttonRecord.clicked.connect(self.record)
        self._widget.buttonRecord.shortcut = QShortcut(QKeySequence("Space"), self._widget)
        self._widget.buttonRecord.shortcut.activated.connect(self.record)
        self._widget.frameList.key_pressed.connect(self.delete)
        self._widget.frameList.itemSelectionChanged.connect(self.frame_select)

    def help(self) -> None:
        """
        Prints out the help dialogue
        """
        message = "This is RecordUI, a tool to record robot animations.\n \n Keyboard shortcuts: \n \n \
            New: Ctrl + N \n \
            Open: Ctrl + O \n \
            Save: Ctrl + S \n \
            Save as: Ctrl + Shift + S \n \n \
            Play Init: Ctrl + I \n \
            Play Frame: Ctrl + P \n \
            Play Next: Alt + P \n \
            Play Animation: Ctrl + Shift + P \n \
            Play Until: Ctrl + Alt + P \n \n \
            Record Frame: Space \n \
            Duplicate Frame: '+' \n \
            Delete Frame: '-' \n \
            Undo: Ctrl + Z \n \
            Redo: Ctrl + Y \n \
            Help: Ctrl + H \n \n \
            Mirror to Left: Ctrl + Left Arrow \n \
            Mirror to Right: Ctrl + Right Arrow \n \
            Invert: Ctrl + Down Arrow"
        QMessageBox.about(self._widget, "About RecordUI", message)

    def new(self) -> None:
        """
        Deletes all currently recorded frames
        """
        if len(self._widget.frameList) > 1:
            message = "This will discard your current Animation. Continue?"
            sure = QMessageBox.question(self._widget, "Sure?", message, QMessageBox.Yes | QMessageBox.No)
            if sure == QMessageBox.Yes:
                # Create a new recorder (deletes all states and starts from scratch)
                self.create_initialized_recorder()
                self.update_frames()

    def create_initialized_recorder(self) -> None:
        """
        Creates a new recorder and adds an initial keyframe to it
        """
        # Create a new recorder (deletes all states and starts from scratch)
        self._recorder = Recorder(
            self._node, self.animation_client, self.add_animation_client, self.hcm_record_mode_client
        )
        # Find a name for the first keyframe
        self._selected_frame = "start frame"
        # Add an initial keyframe to the recorder (it has no motors active, but serves as a starting point)
        self._recorder.record({}, {}, self._selected_frame, 1.0, 0.0, frozen=True)

    def save(self, new_location: bool = False) -> None:
        """
        Saves all currently recorded frames into a json file
        """
        # Check if there is anything to save
        if self._recorder.get_keyframes():
            # Add metadata from UI to the animation
            self._recorder.set_meta_data(
                self._widget.lineAnimationName.text(),
                self._widget.lineVersion.text(),
                self._widget.lineAuthor.text(),
                self._widget.fieldDescription.toPlainText(),
            )
            # Ask for a save location if there is none or if the user wants to save to a new location
            if not self._save_directory or new_location:
                # QFileDialogue throws a gtk warning, that can not be suppressed or fixed. Should be ignored.
                self._save_directory = QFileDialog.getExistingDirectory(
                    self._widget, "Select Directory for Animation Files", os.path.expanduser("~")
                )
            # Save the animation
            status = self._recorder.save_animation(self._save_directory, self._widget.lineAnimationName.text())
            self._widget.statusBar.showMessage(status)
        else:
            self._widget.statusBar.showMessage("There is nothing to save!")

    def open(self) -> None:
        """
        Deletes all current frames and instead loads an animation from a json file
        """
        # Check for unsaved changes and ask the user if they want to discard them if there are any
        if len(self._recorder.get_keyframes()) > 1:
            message = "This will discard your current Animation. Continue?"
            sure = QMessageBox.question(self._widget, "Sure?", message, QMessageBox.Yes | QMessageBox.No)
            # Cancel the open if the user does not want to discard the current animation
            if sure == QMessageBox.No:
                return
        # Open the file dialog in the animations build directory
        my_file = QFileDialog.getOpenFileName(
            directory=os.path.join(get_package_share_directory("wolfgang_animations"), "animations"), filter="*.json"
        )

        # Cancel the open if the user does not select a file
        if not my_file or not my_file[0]:
            return

        # Load the animation
        status = self._recorder.load_animation(my_file[0])

        # Update the UI
        if status == "":
            status = "Load successful."
        self._widget.statusBar.showMessage(status)

        # Update what motor is active and has torque in the UI
        # TODO

        # Update the frames in the UI
        self.update_frames()

        # Update the metadata input in the UI
        metadata = self._recorder.get_meta_data()
        self._widget.lineAnimationName.setText(metadata[0])
        self._widget.lineAuthor.setText(metadata[2])
        self._widget.lineVersion.setText(str(metadata[1]))
        self._widget.fieldDescription.setPlainText(metadata[3])

    def play(self):
        """
        Plays the animation
        """
        status, success = self._recorder.play()
        self._widget.statusBar.showMessage(status)
        if not success:
            QMessageBox.warning(self._widget, "Warning", status)

    def play_until(self):
        """
        Plays the animation up to a certain frame
        """
        # Get the index of the selected frame
        end_index = self._recorder.get_keyframe_index(self._selected_frame) + 1

        # Play the animation
        status, success = self._recorder.play(until_frame=end_index)
        if not success:
            QMessageBox.warning(self._widget, "Warning", status)

    def goto_frame(self):
        """
        Plays one single frame
        """
        index = self._recorder.get_keyframe_index(self._selected_frame)
        assert index is not None, "Selected frame not found in list of keyframes"

        self._recorder.play(from_frame=index, until_frame=index)

    def goto_next(self):
        # Get current index
        index = self._recorder.get_keyframe_index(self._selected_frame)
        assert index is not None, "Selected frame not found in list of keyframes"

        # Check if this is the last frame
        if index + 1 < len(self._recorder.get_keyframes()):
            # Play until the next frame
            self._recorder.play(from_frame=index, until_frame=index + 1)
            # Go to the next frame in the UI if we are not at the end
            self._widget.frameList.setCurrentRow(index + 1)
            self._selected_frame = self._recorder.get_keyframes()[index + 1]["name"]
            self.react_to_frame_change()
        else:
            self.goto_frame()

    def goto_init(self):
        """
        Plays init frame
        """
        # Request record mode from HCM
        self.hcm_record_mode_client.call(SetBool.Request(data=True))
        # Play the init animation
        self.animation_client.send_goal_async(
            PlayAnimation.Goal(
                animation="init",
            )
        )

    def duplicate(self):
        """
        Creates a copy of the selected frame
        """
        try:
            frame = self._widget.frameList.selectedItems()[0].text()
        except Exception as e:
            self._node.get_logger().error(str(e))
            return
        assert self._recorder.get_keyframe(frame) is not None, "Selected frame not found in list of keyframes"
        self._recorder.duplicate(frame)
        self._widget.statusBar.showMessage("Duplicated frame " + frame)
        self.update_frames()

    def delete(self):
        """
        Deletes a frame
        """
        try:
            frame = self._widget.frameList.selectedItems()[0].text()
        except Exception as e:
            self._node.get_logger().error(str(e))
            return
        assert self._recorder.get_keyframe(frame) is not None, "Selected frame not found in list of keyframes"
        self._recorder.delete(frame)
        self._widget.statusBar.showMessage(f"Deleted frame {frame}")
        self.update_frames()

    def record(self):
        """
        Records a frame, meaning it saves the current motor positions and settings in the recorder at the selected frame
        """
        # Get the index of the currently selected frame
        index = self._recorder.get_keyframe_index(self._selected_frame)
        assert index is not None, "Selected frame not found in list of keyframes"

        # Check if we added a valid name
        new_name = self._widget.lineFrameName.text()
        if not new_name:
            QMessageBox.warning(self._widget, "Warning", "Please enter a unique name for the keyframe.")
            return

        # Check if the name is unique
        if new_name != self._selected_frame and self._recorder.get_keyframe(new_name) is not None:
            QMessageBox.warning(self._widget, "Warning", f"A keyframe with the name '{new_name}' already exists.")
            return

        # Record the frame
        self._recorder.record(
            # Only record the active motors
            {
                motor_name: self._working_angles[motor_name]
                for motor_name in self.motors
                if self._motor_switcher_active_checkbox[motor_name].checkState(0) == 2
            },
            {motor_name: self._motor_switcher_torque_checkbox[motor_name].checkState(0) for motor_name in self.motors},
            new_name,
            self._widget.spinBoxDuration.value(),
            self._widget.spinBoxPause.value(),
            index,
            True,
        )

        # Update the selected frame
        self._selected_frame = new_name

        # Display a message
        self._widget.statusBar.showMessage(f"Recorded frame {self._selected_frame}")

        # Update the frames in the UI
        self.update_frames()

    def undo(self):
        """
        Undo the previous action
        """
        status = self._recorder.undo()
        self._widget.statusBar.showMessage(status)
        self.update_frames()

    def redo(self):
        """
        Redos an action
        """
        status = self._recorder.redo()
        self._widget.statusBar.showMessage(status)
        self.update_frames()

    def mirror_frame(self, direction):
        """
        Copies all motor values from one side of the robot to the other. Inverts values, if necessary
        """
        raise NotImplementedError("This function is not yet implemented")
        self._widget.statusBar.showMessage("Mirrored frame to " + direction)

    def invert_frame(self):
        """
        Copies all values from the left side to the right and all values from the right side to the left.
        Inverts values, if necessary
        """
        raise NotImplementedError("This function is not yet implemented")
        self._widget.statusBar.showMessage("Inverted frame")

    def frame_select(self):
        """
        Gets called when a frame is selected in the list of frames
        """
        # Check if a frame is selected at all
        if self._widget.frameList.currentItem() is not None:
            # Get the selected frame
            selected_frame_name = self._widget.frameList.currentItem().text()
            selected_frame = self._recorder.get_keyframe(selected_frame_name)
            if selected_frame is not None:
                # Update state so we have a new selected frame
                self._selected_frame = selected_frame_name

        # Update the UI
        self.react_to_frame_change()

    def react_to_frame_change(self):
        """
        Updates the UI when the frame changes
        """
        # Get the selected frame
        selected_frame = self._recorder.get_keyframe(self._selected_frame)

        # Update the name
        self._widget.lineFrameName.setText(self._selected_frame)

        # Update what motors are active, what are stiff and set the angles accordingly
        for motor_name in self.motors:
            # Update checkbox if the motor is active or not
            active = motor_name in selected_frame["goals"]
            self._motor_switcher_active_checkbox[motor_name].setCheckState(0, Qt.Checked if active else Qt.Unchecked)

            # Update checkbox if the motor is stiff or not
            stiff = "torques" not in selected_frame or (
                motor_name in selected_frame["torques"] and bool(selected_frame["torques"][motor_name])
            )
            self._motor_switcher_torque_checkbox[motor_name].setCheckState(0, Qt.Checked if stiff else Qt.Unchecked)

            # Update the motor angle controls (value and active state)
            if active:
                self._motor_controller_text_fields[motor_name].setText(
                    str(round(math.degrees(selected_frame["goals"][motor_name]), 4))
                )
            else:
                self._motor_controller_text_fields[motor_name].setText("0.0")

        # Update the duration and pause
        self._widget.spinBoxDuration.setValue(selected_frame["duration"])
        self._widget.spinBoxPause.setValue(selected_frame["pause"])

        self.react_to_motor_selection()

    def textfield_update(self):
        """
        If the textfield is updated, update working values
        """
        for motor_name, text_field in self._motor_controller_text_fields.items():
            try:
                # Get the angle from the textfield
                angle = float(text_field.text())
            except ValueError:
                # Display QMessageBox stating that the value is not a number
                QMessageBox.warning(
                    self._widget,
                    "Warning",
                    f"Please enter a valid number.\n '{text_field.text()}' is not a valid number.",
                )
                return
            # Clip the angle to the maximum and minimum, we do this in degrees,
            # because we do not want introduce rounding errors in the textfield
            angle = round(max(-180.0, min(angle, 180.0)), 4)
            # Set the angle in the textfield
            if float(text_field.text()) != float(angle):
                text_field.setText(str(angle))
            # Set the angle in the working values
            self._working_angles[motor_name] = math.radians(angle)

    def react_to_motor_selection(self):
        """
        Fetches which motors are active from the UI. It also fetches whether the motor is stiff or not. It then
        publishes the motor torques to the robot.
        """
        # Go through all motors
        for motor_name in self.motors:
            active = self._motor_switcher_active_checkbox[motor_name].checkState(0) == 2
            # Enable or disable the motor controls
            self._motor_controller_text_fields[motor_name].setEnabled(active)
            # Pull working values from from the joint states if the motor is not active
            if not active and motor_name in self._joint_states.name:
                self._working_angles[motor_name] = self._joint_states.position[
                    self._joint_states.name.index(motor_name)
                ]

        # Publish position message
        self._joint_pub.publish(
            JointCommand(
                # Set set the motor controller constraints to the maximum
                accelerations=[-1.0] * len(self.motors),
                max_currents=[-1.0] * len(self.motors),
                # Set velocities to 1.0 # TODO evaluate
                velocities=[1.0] * len(self.motors),
                # Set the joint names and positions
                joint_names=self._working_angles.keys(),
                positions=self._working_angles.values(),
            )
        )

        # Publish the torque message
        self.effort_pub.publish(
            JointTorque(
                joint_names=self.motors,
                on=[self._motor_switcher_torque_checkbox[motor_name].checkState(0) == 2 for motor_name in self.motors],
            )
        )

    def update_frames(self) -> None:
        """
        Updates the list of frames in the GUI based on the keyframes in the recorder
        """
        # Get the current state of the recorder and the index of the selected frame
        keyframes = self._recorder.get_keyframes()
        index = self._recorder.get_keyframe_index(self._selected_frame)

        # Check if the selected frame is still in the list of frames
        if index is None:
            # If it is not, select the first frame
            index = 0
            self._selected_frame = keyframes[index]["name"]
            print(f"Selected frame {self._selected_frame}")

        # Clear the list of frames
        self._widget.frameList.clear()

        # Add all frames to the list
        for frame in keyframes:
            item = QListWidgetItem()
            item.setText(frame["name"])
            self._widget.frameList.addItem(item)

        # Select the correct frame
        self._widget.frameList.setCurrentRow(index)

        # Variables depending on the frame selection
        self.react_to_frame_change()

    def change_keyframe_order(self, new_order):
        """Calls the recorder to update frame order and updates the gui"""
        self._recorder.change_frame_order(new_order)
        self.update_frames()

    def shutdown_plugin(self):
        """Clean up by sending the HCM that we are not in record mode anymore"""
        self.hcm_record_mode_client.call(SetBool.Request(data=False))


def main():
    plugin = "bitbots_animation_rqt.record_ui.RecordUI"
    main = Main(filename=plugin)
    sys.exit(main.main(standalone=plugin))

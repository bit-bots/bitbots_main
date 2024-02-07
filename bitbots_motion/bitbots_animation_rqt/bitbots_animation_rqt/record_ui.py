#!/usr/bin/env python3
import math
import os
import sys
import time
from copy import deepcopy

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
    QSlider,
    QTreeWidgetItem,
    QVBoxLayout,
)
from rclpy.node import Node
from rqt_gui.main import Main
from rqt_gui_py.plugin import Plugin
from sensor_msgs.msg import JointState

from bitbots_animation_rqt.animation_recording import Recorder
from bitbots_animation_rqt.utils import DragDropList
from bitbots_msgs.msg import JointCommand, JointTorque


class RecordUI(Plugin):
    """
    This class is the main class for the RecordUI. It is a plugin for the rqt framework and is used to record animations.
    """

    def __init__(self, context):
        super().__init__(context)
        self._node: Node = context.node

        # Initialize the window
        self._widget = QMainWindow()

        # Load XML ui definition
        ui_file = os.path.join(get_package_share_directory("bitbots_animation_rqt"), "resource", "RecordUI.ui")
        loadUi(ui_file, self._widget, {})

        self._recorder = Recorder(self._node)
        self._sliders = {}
        self._textFields = {}
        self._motorSwitched = {}
        self._selected_frame = None

        self._currentGoals = {}  # this is the data about the current unsaved frame
        self._currentDuration = 1.0
        self._currentPause = 0.0
        self._currentName = "new frame"

        self._workingValues = {}  # this is the data about the frame that is displayed
        self._workingDuration = 1.0
        self._workingPause = 0.0
        self._workingName = self._currentName

        self._current = True

        self._saveDir = None

        self._robot_anim_path = os.path.join(get_package_share_directory("wolfgang_animations"), "animations")

        # save current frame when switching to other frames for reference
        # working frame

        self._treeItems = {}
        self._motorCheckBody = QTreeWidgetItem(self._widget.motorTree)
        self._motorCheckLegs = QTreeWidgetItem(self._motorCheckBody)
        self._motorCheckArms = QTreeWidgetItem(self._motorCheckBody)
        self._motorCheckHead = QTreeWidgetItem(self._motorCheckBody)
        self._motorCheckLArm = QTreeWidgetItem(self._motorCheckArms)
        self._motorCheckRArm = QTreeWidgetItem(self._motorCheckArms)
        self._motorCheckLLeg = QTreeWidgetItem(self._motorCheckLegs)
        self._motorCheckRLeg = QTreeWidgetItem(self._motorCheckLegs)

        # saves configuration of the trees checkboxes for each of the tree modes.
        self._checkBoxesSave = {}
        self._checkBoxesPower = {}
        self._previousTreeMode = 0

        self.setObjectName("Record Animation")

        self._widget.frameList = DragDropList(self._widget, self)
        self._widget.verticalLayout_2.insertWidget(0, self._widget.frameList)
        self._widget.frameList.setDragDropMode(QAbstractItemView.InternalMove)

        # Create subscriptions
        self.state_sub = self._node.create_subscription(JointState, "joint_states", self.state_update, 1)

        # Create publishers
        self._joint_pub = self._node.create_publisher(JointCommand, "record_motor_goals", 1)
        self.effort_pub = self._node.create_publisher(JointTorque, "ros_control/set_torque_individual", 1)

        # Create a dictionary to map joint names to ids # TODO this should not be hardcoded
        self.ids = {
            "HeadPan": 0,
            "HeadTilt": 1,
            "LShoulderPitch": 2,
            "LShoulderRoll": 3,
            "LElbow": 4,
            "RShoulderPitch": 5,
            "RShoulderRoll": 6,
            "RElbow": 7,
            "LHipYaw": 8,
            "LHipRoll": 9,
            "LHipPitch": 10,
            "LKnee": 11,
            "LAnklePitch": 12,
            "LAnkleRoll": 13,
            "RHipYaw": 14,
            "RHipRoll": 15,
            "RHipPitch": 16,
            "RKnee": 17,
            "RAnklePitch": 18,
            "RAnkleRoll": 19,
        }

        self._initial_joints = JointState()

        self.update_time = 0.1

        for k, _v in self.ids.items():
            self._node.get_logger().info(f"Adding {k} to initial_joints")
            self._initial_joints.name.append(k)
            self._initial_joints.position.append(0)

        for i in range(0, len(self._initial_joints.name)):
            self._currentGoals[self._initial_joints.name[i]] = self._initial_joints.position[i]
            self._workingValues[self._initial_joints.name[i]] = self._initial_joints.position[i]
            self._motorSwitched[self._initial_joints.name[i]] = True

        init_torque = {}
        for k in self._workingValues.keys():
            init_torque[k] = 2

        self._checkBoxesPower["#CURRENT_FRAME"] = init_torque

        self.motor_controller()
        self.motor_switcher()
        self.action_connect()
        self.box_ticked()
        self.frame_list()
        self.update_frames()
        self.set_sliders_and_text_fields(manual=True)

        context.add_widget(self._widget)
        self._widget.statusBar.showMessage("Initialization complete.")

    def state_update(self, joint_states):
        """
        Callback method for /joint_states. Updates the sliders to the actual values of the motors when the robot moves.
        """
        if not self._initial_joints:
            self._initial_joints = joint_states
            time.sleep(1)
        else:
            for i in range(0, len(joint_states.name)):
                if not self._motorSwitched[joint_states.name[i]]:
                    self._workingValues[joint_states.name[i]] = joint_states.position[i]

        time.sleep(self.update_time)
        self.set_sliders_and_text_fields(manual=False)

    def motor_controller(self):
        """
        Sets up the GUI in the middle of the Screen to control the motors.
        Uses self._motorValues to determine which motors are present.
        """
        i = 0
        for k, _v in sorted(self._currentGoals.items()):
            group = QGroupBox()
            slider = QSlider(Qt.Horizontal)
            slider.setTickInterval(1)
            slider.setMinimum(-181)
            slider.setMaximum(181)
            slider.sliderMoved.connect(
                self.slider_update
            )  # This has to  be a sliderMoved signal, since valueChanged is
            # triggered by other sources than user action.
            self._sliders[k] = slider

            textfield = QLineEdit()
            textfield.setText("0")
            textfield.textEdited.connect(self.textfield_update)
            self._textFields[k] = textfield

            label = QLabel()
            label.setText(k)

            layout = QVBoxLayout()
            layout.addWidget(label)
            layout.addWidget(self._sliders[k])
            layout.addWidget(self._textFields[k])
            group.setLayout(layout)
            self._widget.motorControlLayout.addWidget(group, i // 5, i % 5)
            i = i + 1

    def action_connect(self):
        """
        Connects the actions in the top bar to the corresponding functions, and sets their shortcuts
        :return:
        """
        self._widget.actionNew.triggered.connect(self.new)
        self._widget.actionOpen.triggered.connect(self.open)
        self._widget.actionSave.triggered.connect(self.save)
        self._widget.actionSave_as.triggered.connect(self.save_as)
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
        self._widget.frameList.keyPressed.connect(self.delete)
        self._widget.treeModeSelector.currentIndexChanged.connect(self.tree_mode_changed)

    def help(self):
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

    def new(self):
        """
        Deletes all currently recorded frames
        """
        if len(self._widget.frameList) > 1:
            message = "This will discard your current Animation. Continue?"
            sure = QMessageBox.question(self._widget, "Sure?", message, QMessageBox.Yes | QMessageBox.No)
            if sure == QMessageBox.Yes:
                self._recorder.clear()
                self.update_frames()

    def save(self):
        """
        Saves all currently recorded frames into a json file
        """
        if self._recorder.get_animation_state():
            self.tree_mode_changed(self._previousTreeMode)
            self.set_metadata()
            if not self._saveDir:
                # QFileDialogue throws a gtk warning, that can not be suppressed or fixed. Should be ignored.
                self._saveDir = QFileDialog.getExistingDirectory()
            status = self._recorder.save_animation(
                self._saveDir, self._widget.lineAnimationName.text(), self._checkBoxesSave
            )
            self._widget.statusBar.showMessage(status)
        else:
            self._widget.statusBar.showMessage("There is nothing to save!")
            return

    def save_as(self):
        """
        Saves all currently recorded frames into a json file, which is saved at a user specified location
        """
        if self._recorder.get_animation_state():
            self.tree_mode_changed(self._previousTreeMode)
            self._saveDir = QFileDialog.getExistingDirectory()
            self._recorder.save_animation(self._saveDir, self._widget.lineAnimationName.text(), self._checkBoxesSave)
        else:
            self._widget.statusBar.showMessage("There is nothing to save!")
            return

    def set_metadata(self):
        status = self._recorder.set_meta_data(
            self._widget.lineAnimationName.text(),
            self._widget.lineVersion.text(),
            self._widget.lineAuthor.text(),
            self._widget.fieldDescription.toPlainText(),
        )
        self._widget.statusBar.showMessage(status)

    def open(self):
        """
        Deletes all current frames and instead loads an animation from a json file
        """
        if len(self._widget.frameList) > 1:
            message = "This will discard your current Animation. Continue?"
            sure = QMessageBox.question(self._widget, "Sure?", message, QMessageBox.Yes | QMessageBox.No)
            if sure == QMessageBox.No:
                return
        my_file = QFileDialog.getOpenFileName(
            directory=os.path.join(get_package_share_directory("wolfgang_animations"), "animations"), filter="*.json"
        )
        if my_file:
            status = self._recorder.load_animation(my_file)
            if status == "":
                status = "Load successful."
            self._widget.statusBar.showMessage(status)

        animstate = self._recorder.get_animation_state()
        for i in animstate:
            try:
                self._checkBoxesPower[i["name"]] = i["torque"]
            except KeyError:
                self._checkBoxesPower[i["name"]] = {}
                for key in self.ids:
                    self._checkBoxesPower[i["name"]][key] = 2

        self.update_frames()

        metadata = self._recorder.get_meta_data()

        self._widget.lineAnimationName.setText(metadata[0])
        self._widget.lineAuthor.setText(metadata[2])
        self._widget.lineVersion.setText(str(metadata[1]))
        self._widget.fieldDescription.setPlainText(metadata[3])

    def play(self):
        """
        Plays the animation
        """
        status = self._recorder.play(self._robot_anim_path)
        self._widget.statusBar.showMessage(status)

    def play_until(self):
        """
        Plays the animation up to a certain frame
        """
        steps = self._recorder.get_animation_state()
        j = 0
        for i in range(0, len(steps)):
            j += 1
            self._node.get_logger().info(steps[i]["name"])
            if steps[i]["name"] == self._selected_frame["name"]:
                break
        self._recorder.play(self._robot_anim_path, j)

    def goto_frame(self):
        """
        Plays one single frame
        """
        self.set_all_joints_stiff()

        pos_msg = JointCommand()
        pos_msg.joint_names = []
        pos_msg.velocities = [1.0] * 20
        pos_msg.positions = []
        pos_msg.accelerations = [-1.0] * 20
        pos_msg.max_currents = [-1.0] * 20

        for k, v in self._workingValues.items():
            pos_msg.joint_names.append(k)
            pos_msg.positions.append(v)

        torque_msg = JointTorque()
        torque_msg.joint_names = []
        torque_msg.on = []

        for k, v in self._checkBoxesPower[self._widget.frameList.currentItem().text()].items():
            torque_msg.joint_names.append(k)
            torque_msg.on.append(v)

        self.effort_pub.publish(torque_msg)
        self._joint_pub.publish(pos_msg)

    def goto_next(self):
        if self._widget.frameList.currentRow() < self._widget.frameList.count() - 2:
            self._widget.frameList.setCurrentRow(self._widget.frameList.currentRow() + 1)
            self.goto_frame()

    def goto_init(self):
        """
        Plays init frame, sets all motor values to 0
        """
        self.set_all_joints_stiff()

        if self._widget.frameList.currentItem().text() == "#CURRENT_FRAME":
            for k, _v in self._workingValues.items():
                self._workingValues[k] = 0.0
        for k, _v in self._currentGoals.items():
            self._currentGoals[k] = 0.0
        self.set_sliders_and_text_fields(manual=False)

        pos_msg = JointCommand()
        pos_msg.joint_names = self._initial_joints.name
        pos_msg.velocities = [1.0] * len(self._initial_joints.name)
        pos_msg.positions = [0.0] * len(self._initial_joints.name)
        pos_msg.accelerations = [-1.0] * len(self._initial_joints.name)
        pos_msg.max_currents = [-1.0] * len(self._initial_joints.name)

        self._joint_pub.publish(pos_msg)

    def set_all_joints_stiff(self):
        """
        Enables torque for all motors
        """
        if self._widget.frameList.currentItem().text() == "#CURRENT_FRAME":
            for v in self._treeItems.values():
                v.setCheckState(0, Qt.Checked)

        torque_msg = JointTorque()
        torque_msg.joint_names = []
        torque_msg.on = []

        for k in sorted(self._treeItems.keys()):
            torque_msg.joint_names.append(k)
            torque_msg.on.append(True)

        self.effort_pub.publish(torque_msg)

    def duplicate(self):
        """
        Creates a copy of the selected frame
        """
        try:
            frame = self._widget.frameList.selectedItems()[0].text()
        except Exception as e:
            self._node.get_logger().error(str(e))
            return
        if frame:
            if not frame == "#CURRENT_FRAME":
                self._recorder.duplicate(frame)
                self._widget.statusBar.showMessage("Duplicated frame " + frame)
            else:
                self._widget.statusBar.showMessage("Cannot duplicate current frame. Record first.")
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
        if frame:
            if not frame == "#CURRENT_FRAME":
                self._recorder.delete(frame)
                self._widget.statusBar.showMessage("Deleted frame " + frame)
            else:
                self._widget.statusBar.showMessage("Cannot delete current frame.")
            self.update_frames()

    def record(self, keep=False):
        """
        Records a frame
        """
        if self._widget.frameList.currentItem() is not None:
            if self._widget.frameList.currentItem().text() == "#CURRENT_FRAME":
                x = True
                n = 0
                for state in self._recorder.get_animation_state():
                    if self._workingName == state["name"]:
                        while x:
                            x = False
                            for state in self._recorder.get_animation_state():
                                if self._workingName + str(n) == state["name"]:
                                    n += 1
                                    x = True
                        self._workingName = self._workingName + str(n)

                self.set_sliders_and_text_fields(manual=True)

                self._checkBoxesPower[self._workingName] = {}
                init_torque = {}
                for k, _v in self._workingValues.items():
                    init_torque[k] = 2

                self._checkBoxesPower[self._workingName] = init_torque

                self._recorder.record(
                    self._workingValues,
                    init_torque,
                    self._widget.lineFrameName.text(),
                    self._widget.spinBoxDuration.value(),
                    self._widget.spinBoxPause.value(),
                )
                self._currentGoals = deepcopy(self._workingValues)
            else:
                current_row = self._widget.frameList.currentRow()

                self._recorder.record(
                    self._workingValues,
                    self._checkBoxesPower[self._widget.frameList.currentItem().text()],
                    self._widget.lineFrameName.text(),
                    self._widget.spinBoxDuration.value(),
                    self._widget.spinBoxPause.value(),
                    current_row,
                    True,
                )

            self._widget.statusBar.showMessage("Recorded frame " + self._workingName)
        self.update_frames(keep)

    def undo(self):
        """
        Undos the previous action
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
        opposite = "L"
        if direction == "L":
            opposite = "R"
        for k, v in self._workingValues.items():
            if k[0] == opposite:
                for k1, _v1 in self._workingValues.items():
                    if k1 == str(direction) + k[1:]:
                        self._workingValues[k1] = v * -1

        boxmode = 0
        if self._widget.treeModeSelector.currentIndex() == 0:
            boxmode = self._checkBoxesPower[self._widget.frameList.currentItem().text()]
        elif self._widget.treeModeSelector.currentIndex() == 1:
            boxmode = self._checkBoxesSave[self._widget.frameList.currentItem().text()]

        for k in boxmode.keys():
            if k[0] == opposite:
                for k1 in boxmode.keys():
                    if k1 == str(direction) + k[1:]:
                        if boxmode[k] == 0:
                            boxmode[k1] = 0
                        elif boxmode[k] == 2:
                            boxmode[k1] = 2

        self.update_tree_config(self._widget.treeModeSelector.currentIndex())
        self.box_ticked()
        self._widget.statusBar.showMessage("Mirrored frame to " + direction)

    def invert_frame(self):
        """
        Copies all values from the left side to the right and all values from the right side to the left.
        Inverts values, if necessary
        """
        temp_dict = {}
        for k, v in self._workingValues.items():
            if k[0] == "L":
                temp_dict["R" + k[1:]] = -v
            elif k[0] == "R":
                temp_dict["L" + k[1:]] = -v
        for k, v in temp_dict.items():
            self._workingValues[k] = v

        boxmode = 0
        if self._widget.treeModeSelector.currentIndex() == 0:
            boxmode = self._checkBoxesPower
        elif self._widget.treeModeSelector.currentIndex() == 1:
            boxmode = self._checkBoxesSave

        for k, v in boxmode[self._widget.frameList.currentItem().text()].items():
            if k[0] == "L":
                if v == 2:
                    temp_dict["R" + k[1:]] = 2
                elif v == 0:
                    temp_dict["R" + k[1:]] = 0
            elif k[0] == "R":
                if v == 2:
                    temp_dict["L" + k[1:]] = 2
                elif v == 0:
                    temp_dict["L" + k[1:]] = 0

        boxmode[self._widget.frameList.currentItem().text()] = deepcopy(temp_dict)
        self.update_tree_config(self._widget.treeModeSelector.currentIndex())
        self.box_ticked()
        self._widget.statusBar.showMessage("Inverted frame")

    def frame_list(self):
        """
        Connects triggers from the frame list to their callback methods.
        """
        self._widget.frameList.itemSelectionChanged.connect(self.frame_select)
        self._widget.lineFrameName.textEdited.connect(self.frame_meta_update)
        self._widget.spinBoxPause.valueChanged.connect(self.frame_meta_update)
        self._widget.spinBoxDuration.valueChanged.connect(self.frame_meta_update)

    def frame_meta_update(self):
        """
        Updates the respective values for the two spin boxes and the frame name, when they are changed
        """
        self._workingDuration = self._widget.spinBoxDuration.value()
        self._workingPause = self._widget.spinBoxPause.value()
        self._workingName = self._widget.lineFrameName.text()

    def frame_select(self):
        """
        Loads all information on a specific frame into the working values, if the frame selection changes
        """
        if self._widget.frameList.currentItem() is not None:
            self.copy_old_tree_config()
            selected_frame_name = self._widget.frameList.currentItem().text()
            self._selected_frame = None

            for v in []:  # TODO self._recorder.get_animation_state():
                if v["name"] == selected_frame_name:
                    self._selected_frame = v
                    break

            # save current values to _currentValues if switching from current frame to different one
            # when switching from current to different, save current values

            if selected_frame_name == "#CURRENT_FRAME":
                self._widget.treeModeSelector.setCurrentIndex(0)
                self.update_tree_config(0)
                self._widget.treeModeSelector.setEnabled(False)
                self._workingValues = deepcopy(self._currentGoals)
                self._workingName = deepcopy(self._currentName)
                self._workingDuration = deepcopy(self._currentDuration)
                self._workingPause = deepcopy(self._currentPause)

                self._current = True
            else:
                if self._selected_frame is None:
                    return
                self._widget.treeModeSelector.setEnabled(True)
                if self._current:
                    self._currentGoals = deepcopy(self._workingValues)
                    self._currentName = deepcopy(self._workingName)
                    self._currentDuration = deepcopy(self._workingDuration)
                    self._currentPause = deepcopy(self._workingPause)

                self._workingValues = self._selected_frame["goals"]
                self._workingName = self._selected_frame["name"]
                self._workingPause = self._selected_frame["pause"]
                self._workingDuration = float(self._selected_frame["duration"])

                self._widget.lineFrameName.setText(self._widget.frameList.currentItem().text())
                self._current = False
                self.update_tree_config(self._previousTreeMode)

            if self._widget.treeModeSelector.currentIndex() == 0:
                for k, v in self._treeItems.items():
                    self._motorSwitched[k] = v.checkState(0) == 2

            for k, v in self._motorSwitched.items():
                self._textFields[k].setEnabled(v)
                self._sliders[k].setEnabled(v)

        self.box_ticked()

    def motor_switcher(self):
        """
        Loads the motors into the tree and adds the checkboxes
        """
        self._widget.motorTree.setHeaderLabel("Stiff Motors")
        self._motorCheckBody.setText(0, "Body")
        self._motorCheckBody.setFlags(self._motorCheckBody.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckBody.setExpanded(True)
        self._motorCheckHead.setText(0, "Head")
        self._motorCheckHead.setFlags(self._motorCheckHead.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckHead.setExpanded(True)
        self._motorCheckArms.setText(0, "Arms")
        self._motorCheckArms.setFlags(self._motorCheckArms.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckArms.setExpanded(True)
        self._motorCheckLegs.setText(0, "Legs")
        self._motorCheckLegs.setFlags(self._motorCheckLegs.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckLegs.setExpanded(True)
        self._motorCheckLArm.setText(0, "Left Arm")
        self._motorCheckLArm.setFlags(self._motorCheckLArm.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckLArm.setExpanded(True)
        self._motorCheckRArm.setText(0, "Right Arm")
        self._motorCheckRArm.setFlags(self._motorCheckRArm.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckRArm.setExpanded(True)
        self._motorCheckLLeg.setText(0, "Left Leg")
        self._motorCheckLLeg.setFlags(self._motorCheckLLeg.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckLLeg.setExpanded(True)
        self._motorCheckRLeg.setText(0, "Right Leg")
        self._motorCheckRLeg.setFlags(self._motorCheckRLeg.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckRLeg.setExpanded(True)

        for k, _v in self._currentGoals.items():
            parent = None
            if "LHip" in k or "LKnee" in k or "LAnkle" in k:
                parent = self._motorCheckLLeg
            elif "RHip" in k or "RKnee" in k or "RAnkle" in k:
                parent = self._motorCheckRLeg
            elif "LShoulder" in k or "LElbow" in k:
                parent = self._motorCheckLArm
            elif "RShoulder" in k or "RElbow" in k:
                parent = self._motorCheckRArm
            elif "Head" in k:
                parent = self._motorCheckHead
            child = QTreeWidgetItem(parent)
            child.setText(0, k)
            child.setFlags(child.flags() | Qt.ItemIsUserCheckable)
            child.setCheckState(0, Qt.Checked)
            self._treeItems[k] = child

        self._widget.motorTree.itemClicked.connect(self.box_ticked)

    def copy_old_tree_config(self):
        """
        Saves the current configuration of the motor tree checkboxes into the corresponding dictionary
        """
        if self._selected_frame is not None:
            temp_dict = {}
            for k in self._treeItems.keys():
                temp_dict[k] = self._treeItems[k].checkState(0)

            if self._previousTreeMode == 1:
                self._checkBoxesSave[self._selected_frame["name"]] = deepcopy(temp_dict)
            elif self._previousTreeMode == 0:
                self._checkBoxesPower[self._selected_frame["name"]] = deepcopy(temp_dict)

    def update_tree_config(self, index):
        """
        Loads the new configuration of the motor tree depending on the change
        """
        temp_dict_2 = {}
        if self._widget.frameList.currentItem() is not None:
            if not self._widget.frameList.currentItem().text() == "#CURRENT_FRAME":
                if index == 1:
                    if self._selected_frame["name"] in self._checkBoxesSave.keys():
                        temp_dict_2 = deepcopy(self._checkBoxesSave[self._selected_frame["name"]])
                    else:
                        for k in self._workingValues:
                            temp_dict_2[k] = 2
                elif index == 0:
                    if self._selected_frame["name"] in self._checkBoxesPower.keys():
                        temp_dict_2 = deepcopy(self._checkBoxesPower[self._selected_frame["name"]])
                    else:
                        for k in self._workingValues:
                            temp_dict_2[k] = 2
            else:
                for k in self._workingValues:
                    temp_dict_2[k] = 2
            self._previousTreeMode = index
            for k, v in temp_dict_2.items():
                if v == 0:
                    self._treeItems[k].setCheckState(0, Qt.Unchecked)
                elif v == 1:
                    self._treeItems[k].setCheckState(0, Qt.PartiallyChecked)
                elif v == 2:
                    self._treeItems[k].setCheckState(0, Qt.Checked)

    def tree_mode_changed(self, index):
        self.copy_old_tree_config()
        self.update_tree_config(index)

    def slider_update(self):
        """
        Updates all sliders, checks whether a value is too large, then replaces it
        """
        for k, v in self._sliders.items():
            self._workingValues[k] = math.radians(v.value())
            if self._workingValues[k] < -math.pi:
                self._workingValues[k] = -math.pi
            elif self._workingValues[k] > math.pi:
                self._workingValues[k] = math.pi
        self.set_sliders_and_text_fields(manual=True)

    def textfield_update(self):
        """
        Updates all textfields.
        """
        for k, v in self._textFields.items():
            try:
                self._workingValues[k] = math.radians(float(v.text()))
            except ValueError:
                continue
        self.set_sliders_and_text_fields(manual=True)

    def set_sliders_and_text_fields(self, manual):
        """
        Updates the text fields and sliders in self._sliders and self._textfields and also frame name and duration and pause
        to the values in self._workingValues.
        """
        for k, v in self._workingValues.items():
            try:
                if not self._treeItems[k].checkState(0) == 0:
                    self._textFields[k].setText(str(int(round(math.degrees(v)))))
                    self._sliders[k].setValue(int(round(math.degrees(v))))

            except KeyError:
                message = "Found a key-value pair for motor (%s), which doesn't seem to exist (yet). Ignoring it." % k
                self._node.get_logger().warn(message)
                self._widget.statusBar.showMessage(message)
                continue
            except RuntimeError:
                message = (
                    "Tried to access a PyQt element that no longer exists. This happens when you close the framework."
                )
                self._node.get_logger().warn(message)
                self._widget.statusBar.showMessage(message)
                break
        if manual:
            self._widget.lineFrameName.setText(self._workingName)
            self._widget.spinBoxDuration.setValue(self._workingDuration)
            self._widget.spinBoxPause.setValue(self._workingPause)

    def box_ticked(self):
        """
        Controls whether a checkbox has been clicked, and reacts.
        """
        for k, v in self._treeItems.items():
            self._motorSwitched[k] = v.checkState(0) == 2

        for k, v in self._motorSwitched.items():
            self._textFields[k].setEnabled(v)
            self._sliders[k].setEnabled(v)

        self.set_sliders_and_text_fields(manual=False)

        torque_msg = JointTorque()
        torque_msg.joint_names = []
        torque_msg.on = []

        if self._widget.treeModeSelector.currentIndex() == 0:
            for k, v in sorted(self._treeItems.items()):
                torque_msg.joint_names.append(k)
                torque_msg.on.append(v.checkState(0) == 2)

        pos_msg = JointCommand()
        pos_msg.joint_names = []
        pos_msg.velocities = [1.0] * 20
        pos_msg.positions = []
        pos_msg.accelerations = [-1.0] * 20
        pos_msg.max_currents = [-1.0] * 20

        for k, v in self._workingValues.items():
            pos_msg.joint_names.append(k)
            pos_msg.positions.append(v)

        self._joint_pub.publish(pos_msg)
        self.effort_pub.publish(torque_msg)
        if self._widget.frameList.currentItem() is not None:
            if not self._widget.frameList.currentItem().text() == "#CURRENT_FRAME":
                self.tree_mode_changed(self._widget.treeModeSelector.currentIndex())

    def update_frames(self, keep=False):
        """
        updates the list of frames present in the current animation
        :return:
        """
        current_index = self._widget.frameList.currentIndex()
        current_mode = self._widget.treeModeSelector.currentIndex()
        current_state = []  # TODO self._recorder.get_animation_state()
        while self._widget.frameList.takeItem(0):
            continue

        for i in range(0, len(current_state)):
            item = QListWidgetItem()
            if "name" in current_state[i]:
                item.setText(current_state[i]["name"])
            else:  # older animations without names for the frames
                item.setText(str(i))
            self._widget.frameList.addItem(item)

        current = QListWidgetItem()
        current.setText("#CURRENT_FRAME")
        self._widget.frameList.addItem(current)
        if keep:
            self._widget.frameList.setCurrentIndex(current_index)
            self._widget.treeModeSelector.setCurrentIndex(current_mode)
        else:
            self._widget.frameList.setCurrentItem(current)
            self._current = True

    def change_frame_order(self, new_order):
        """Calls the recorder to update frame order and updates the gui"""
        self._recorder.change_frame_order(new_order)
        self.update_frames()

    def shutdown_plugin(self):
        """Clean up by sending the HCM a signal that we are no longer recording"""
        self._joint_pub.publish(JointCommand())
        time.sleep(0.5)


def main():
    plugin = "bitbots_animation_rqt.record_ui.RecordUI"
    main = Main(filename=plugin)
    sys.exit(main.main(standalone=plugin))

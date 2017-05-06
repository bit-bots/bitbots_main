#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospkg
import rospy
import time

from python_qt_binding.QtCore import Qt, QMetaType, QDataStream, QVariant
from python_qt_binding import loadUi
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QTreeWidget, QTreeWidgetItem,QListWidgetItem, QSlider, QGroupBox, QVBoxLayout, QLabel, QLineEdit, QListWidget, QAbstractItemView, QFileDialog
from python_qt_binding.QtGui import QDoubleValidator

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import os

from .animation_recording import Recorder

class DragDropList(QListWidget):
    """ QListWidget with an event that is called when a drag and drop action was performed."""

    def __init__(self, parent, ui):
        super(DragDropList, self).__init__(parent)

        self.ui = ui
        self.setAcceptDrops(True)


    def dropEvent(self, e):
        super(DragDropList, self).dropEvent(e)
        items = []
        for i in range(0, self.count()):
            items.append(self.item(i).text())
        print(items)
        self.ui.change_frame_order(items)

class RecordUI(Plugin):
    def __init__(self, context):
        super(RecordUI, self).__init__(context)


        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('bitbots_recordui_rqt'), 'resource', 'RecordUI.ui')
        loadUi(ui_file, self._widget, {})

        self._recorder = Recorder()
        self._sliders = {}
        self._textFields = {}
        self._motorSwitched = {}

        self._currentGoals = {}                 # this is the data about the current unsaved frame
        self._currentDuration = 1.0
        self._currentPause = 0.0
        self._currentName = "new frame"

        self._workingValues = {}                # this is the data about the frame that is displayed
        self._workingDuration = 1.0
        self._workingPause = 0.0
        self._workingName = self._currentName

        self._freeze = False
        self._saveDir = None

        #save current frame when switching to other frames for reference
        #working frame

        self._treeItems = {}
        self._motorCheckBody = QTreeWidgetItem(self._widget.motorTree)
        self._motorCheckLegs = QTreeWidgetItem(self._motorCheckBody)
        self._motorCheckArms = QTreeWidgetItem(self._motorCheckBody)
        self._motorCheckHead = QTreeWidgetItem(self._motorCheckBody)
        self._motorCheckLArm = QTreeWidgetItem(self._motorCheckArms)
        self._motorCheckRArm = QTreeWidgetItem(self._motorCheckArms)
        self._motorCheckLLeg = QTreeWidgetItem(self._motorCheckLegs)
        self._motorCheckRLeg = QTreeWidgetItem(self._motorCheckLegs)

        self.setObjectName('Record Animation')

        self._initial_joints = None

        self._widget.frameList = DragDropList(self._widget, self)
        self._widget.verticalLayout_2.insertWidget(0, self._widget.frameList)
        self._widget.frameList.setDragDropMode(QAbstractItemView.InternalMove)

        rospy.Subscriber("/joint_states", JointState, self.state_update, queue_size=100)
        self._joint_pub = rospy.Publisher("/motor_goals", JointTrajectory, queue_size=1)

        while not self._initial_joints:
            if not rospy.is_shutdown():
                time.sleep(0.5)
                print "wait"
            else:
                return

        self.initialize()

        context.add_widget(self._widget)

    def initialize(self):
        for i in range(0, len(self._initial_joints.name)):
            self._currentGoals[self._initial_joints.name[i]] = self._initial_joints.position[i]
            self._motorSwitched[self._initial_joints.name[i]] = True


        self.motor_controller()
        self.motor_switcher()
        self.button_connect()
        self.box_ticked()
        self.frame_list()
        self.update_frames()

    def state_update(self, joint_states):
        if not self._initial_joints:
            self._initial_joints = joint_states
            time.sleep(1)

        for i in range(0, len(joint_states.name)):
            if (not self._motorSwitched[joint_states.name[i]]) and (not self._freeze):
                self._currentGoals[joint_states.name[i]] = joint_states.position[i]
                print "update"

        self.set_sliders_and_text_fields()

    def motor_controller(self):
        """
        Sets up the GUI in the middle of the Screen to control the motors. 
        Uses self._motorValues to determine which motors are present.
        """
        i = 0
        for k, v in self._currentGoals.items():
            group = QGroupBox()
            slider = QSlider(Qt.Horizontal)
            slider.setTickInterval(1)
            slider.setMinimum(-180)
            slider.setMaximum(180)
            slider.valueChanged.connect(self.slider_update)
            self._sliders[k] = slider

            textfield = QLineEdit()
            textfield.setText('0')
            textfield.textEdited.connect(self.textfield_update)
            textfield.setValidator(QDoubleValidator(-180.0, 180.0, 2))
            self._textFields[k] = textfield

            label = QLabel()
            label.setText(k)

            layout = QVBoxLayout()
            layout.addWidget(label)
            layout.addWidget(self._sliders[k])
            layout.addWidget(self._textFields[k])
            group.setLayout(layout)
            self._widget.motorControlLayout.addWidget(group, i / 5, i % 5)
            i = i+1

    def button_connect(self):
        """
        Connects the buttons in the top bar to the corresponding functions
        :return: 
        """
        self._widget.buttonNew.clicked.connect(self.new)
        self._widget.buttonSave.clicked.connect(self.save)
        self._widget.buttonSaveAs.clicked.connect(self.save_as)
        self._widget.buttonOpen.clicked.connect(self.open)


        self._widget.buttonPlay.clicked.connect(self.play)
        self._widget.buttonGotoFrame.clicked.connect(self.goto_frame)
        self._widget.buttonRecord.clicked.connect(self.record)

        self._widget.buttonDuplicateFrame.clicked.connect(self.duplicate)

        self._widget.buttonUndo.clicked.connect(self.undo)
        self._widget.buttonRedo.clicked.connect(self.redo)

    def new(self):
        self._recorder.clear()
        self.update_frames()

    def save(self):
        self.set_metadata()
        if not self._saveDir:
            self._saveDir = QFileDialog.getExistingDirectory()
        self._recorder.save_animation(self._saveDir, self._widget.lineAnimationName.text())

    def save_as(self):
        self._saveDir = QFileDialog.getExistingDirectory()
        self._recorder.save_animation(self._saveDir, self._widget.lineAnimationName.text())

    def set_metadata(self):
        self._recorder.set_meta_data(self._widget.lineAnimationName.text(),
                                     self._widget.lineVersion.text(),
                                     self._widget.lineAuthor.text(),
                                     self._widget.fieldDescription.toPlainText())

    def open(self):
        my_file = QFileDialog.getOpenFileName()[0]
        if my_file:
            self._recorder.load_animation(my_file)
        self.update_frames()

        metadata = self._recorder.get_meta_data()

        self._widget.lineAnimationName.setText(metadata[0])
        self._widget.lineAuthor.setText(metadata[2])
        self._widget.lineVersion.setText(metadata[1])
        self._widget.fieldDescription.setPlainText(metadata[3])

    def play(self):
        self._recorder.play()

    def goto_frame(self):
        raise NotImplementedError
        #todo publish joint trajecory message with stuff

    def duplicate(self):
        frame = self._widget.frameList.selectedItems().text()
        if frame:
            self._recorder.duplicate(frame)
            self.update_frames()

    def record(self):
        self._recorder.record(self._currentGoals, self._widget.lineFrameName.text(), 1, 0) #todo time and pause
        self.update_frames()

    def undo(self):
        self._recorder.undo()
        self.update_frames()

    def redo(self):
        self._recorder.redo()
        self.update_frames()

    def frame_list(self):
        self._widget.frameList.itemClicked.connect(self.frame_select)

    def frame_select(self):
        selected_frame_name = self._widget.frameList.currentItem().text()
        selected_frame = None
        for v in self._recorder.get_animation_state():
            if v["name"] == selected_frame_name:
                selected_frame = v
                break

        self._freeze = not (selected_frame_name == "#CURRENT_FRAME")
        print "freeze is " + str(not (selected_frame_name == "#CURRENT_FRAME"))

        if selected_frame_name != "#CURRENT_FRAME":
            self._currentGoals = selected_frame["goals"]
            print "setting motor values"
            for k,v in self._currentGoals.items():
                print k + " set to " + str(v)
                print str(selected_frame["goals"][k])

        self.set_sliders_and_text_fields()

    def motor_switcher(self):
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

        for k, v in self._currentGoals.items():
            parent = None
            if 'LHip' in k or 'LKnee' in k or 'LAnkle' in k:
                parent = self._motorCheckLLeg
            elif 'RHip' in k or 'RKnee' in k or 'RAnkle' in k:
                parent = self._motorCheckRLeg
            elif 'LShoulder' in k or 'LElbow' in k:
                parent = self._motorCheckLArm
            elif 'RShoulder' in k or 'RElbow' in k:
                parent = self._motorCheckRArm
            elif 'Head' in k:
                parent = self._motorCheckHead
            child = QTreeWidgetItem(parent)
            child.setText(0, k)
            child.setFlags(child.flags() | Qt.ItemIsUserCheckable)
            child.setCheckState(0, Qt.Checked)
            self._treeItems[k] = child

        self._widget.motorTree.itemChanged.connect(self.box_ticked)

    def slider_update(self):
        for k, v in self._sliders.items():
            self._currentGoals[k] = float(v.value()) * 3.14 / 180.0
        self.set_sliders_and_text_fields()

    def set_sliders_and_text_fields(self):
        """
        Updates the text fields and sliders ins self._sliders and self._textfields to the values in self._motor_values
        :return: 
        """
        for k, v in self._currentGoals.items():
            self._textFields[k].setText(str(int(v / 3.14 * 180)))
            self._sliders[k].setValue(int(v / 3.14 * 180))

    def textfield_update(self):
        for k, v in self._textFields.items():
            try:
                self._currentGoals[k] = float(v.text()) * 3.14 / 180.0
            except ValueError:
                continue
        self.set_sliders_and_text_fields()

    def box_ticked(self):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = []
        msg.points = []
        msg.points.append(JointTrajectoryPoint())

        for k, v in self._treeItems.items():
            self._motorSwitched[k] = (v.checkState(0) == Qt.Checked)
            msg.joint_names.append(k)
            msg.points[0].positions.append(self._currentGoals[k])
            if self._motorSwitched[k]:
                msg.points[0].effort.append(1.0)
            else:
                msg.points[0].effort.append(0.0)
        self._joint_pub.publish(msg)
        print "published"

    def update_frames(self):
        """
        updates the list of frames present in the current animation
        :return: 
        """
        current_state = self._recorder.get_animation_state()
        while self._widget.frameList.takeItem(0):
            continue

        for i in range(0, len(current_state)):
            item = QListWidgetItem()
            if "name" in current_state[i]:
                item.setText(current_state[i]["name"])
            else: #older animations without names for the frames
                item.setText(str(i))
            self._widget.frameList.addItem(item)

        current = QListWidgetItem()
        current.setText("#CURRENT_FRAME")
        self._widget.frameList.setCurrentItem(current)

    def change_frame_order(self, new_order):
        """ Calls the recorder to update frame order and updates the gui"""
        self._recorder.change_frame_order(new_order)
        self.update_frames()
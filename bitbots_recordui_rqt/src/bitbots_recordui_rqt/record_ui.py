#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospkg
import rospy
import time

from python_qt_binding.QtCore import Qt
from python_qt_binding import loadUi
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QTreeWidget, QTreeWidgetItem, QSlider, QGroupBox, QVBoxLayout, QLabel, QLineEdit
from python_qt_binding.QtGui import QDoubleValidator

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import os

from .animation_recording import Recorder


class RecordUI(Plugin):
    def __init__(self, context):
        print "super"
        super(RecordUI, self).__init__(context)


        self._widget = QWidget()
        print "thingies"
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('bitbots_recordui_rqt'), 'resource', 'RecordUI.ui')
        print "load ui"
        loadUi(ui_file, self._widget, {})

        self._recorder = Recorder()
        self._sliders = {}
        self._textFields = {}
        self._motorValues = {}
        self._motorSwitched = {}

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


        rospy.Subscriber("/joint_states", JointState, self.state_update, queue_size=100)
        self._joint_pub = rospy.Publisher("/motor_goals", JointTrajectory, queue_size=1)

        while not self._initial_joints or not rospy.is_shutdown():
            time.sleep(0.5)
            print "wait"

        self.initialize()


        context.add_widget(self._widget)

    def initialize(self):
        self.motor_controller()
        self.motor_switcher()
        for i in range(0, len(self._initial_joints.name)):
            self._motorSwitched[self._initial_joints.name[i]] = True
            self._textFields[self._initial_joints.name[i]].setText(self._initial_joints.position[i])
        self.button_connect()


    def state_update(self, joint_states):
        if not self._initial_joints:
            self._initial_joints = joint_states
            time.sleep(1)

        for i in range(0, len(joint_states.name)):
            self._motorValues[joint_states.name[i]] = joint_states.position[i]

        for k, v in self._motorValues.\
                items():
            if not self._motorSwitched[k]:
                self._sliders[k].setValue(v*180/3.14)

    def motor_controller(self):
        for i in range(0, len(self._initial_joints.name)):
            self._motorValues[self._initial_joints.name[i]] = self._initial_joints.position[i]

        i = 0
        for k, v in self._motorValues.items():
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
        self._widget.buttonNew.clicked.connect(self.new)
        self._widget.buttonSave.clicked.connect(self.save)
        self._widget.buttonSaveAs.clicked.connect(self.save_as)
        self._widget.buttonSaveAs.clicked.connect(self.save_as)


        self._widget.buttonPlay.clicked.connect(self.play)
        self._widget.buttonGotoFrame.clicked.connect(self.goto_frame)
        self._widget.buttonRecord.clicked.connect(self.record)

        self._widget.buttonUndo.clicked.connect(self.undo)
        self._widget.buttonRedo.clicked.connect(self.redo)

    def new(self):
        raise NotImplementedError

    def save(self):
        raise NotImplementedError

    def save_as(self):
        raise NotImplementedError

    def open(self):
        raise NotImplementedError

    def play(self):
        raise NotImplementedError

    def goto_frame(self):
        raise NotImplementedError

    def record(self):
        self._recorder.record()

    def undo(self):
        raise NotImplementedError

    def redo(self):
        raise NotImplementedError

    def motor_switcher(self):
        self._widget.motorTree.setHeaderLabel("Motors")
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

        for k, v in self._motorValues.items():
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
            child.setFlags(child.flags() |  Qt.ItemIsUserCheckable)
            child.setCheckState(0, Qt.Checked)
            self._treeItems[k] = child

        self._widget.motorTree.itemChanged.connect(self.box_ticked)

    def slider_update(self):
        for k, v in self._sliders.items():
            self._motorValues[k] = float(v.value())*3.14/180.0
            self._textFields[k].setText(str(v.value()))

    def textfield_update(self):
        for k, v in self._sliders.items():
            try:
                self._motorValues[k] = float(self._textFields[k].text()) * 3.14 / 180.0
                v.setValue(int(self._textFields[k].text()))
            except ValueError:
                continue

    def box_ticked(self):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = []
        msg.points = []
        msg.points.append(JointTrajectoryPoint())

        for k, v in self._treeItems.items():
            self._motorSwitched[k] = (v.checkState(0) == Qt.Checked)
            msg.joint_names.append(k)
            msg.points[0].positions.append(self._motorValues[k])
            if self._motorSwitched[k]:
                msg.points[0].effort.append(1.0)
            else:
                msg.points[0].effort.append(0.0)
        self._joint_pub.publish(msg)
        print "published"


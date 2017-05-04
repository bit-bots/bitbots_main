#!/usr/bin/env python3
import rospkg
import rospy
import python_qt_binding
import time

from python_qt_binding.QtCore import Qt
from python_qt_binding import loadUi
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QTreeWidget, QTreeWidgetItem, QSlider, QGroupBox, QVBoxLayout, QLabel, QLineEdit
from python_qt_binding.QtGui import QDoubleValidator

from sensor_msgs.msg import JointState

import os


class RecordUI(Plugin):
    def __init__(self, context):
        super(RecordUI, self).__init__(context)


        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('bitbots_recordui_rqt'), 'resource', 'RecordUI.ui')
        loadUi(ui_file, self._widget, {})

        self._sliders = {}
        self._textFields = {}
        self._motorValues = {}
        self._treeItems = {}
        self._motorCheckBody = QTreeWidgetItem(self._widget.treeWidget)
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

        while not self._initial_joints:
            time.sleep(0.5)
            print "wait"

        self.initialize()

        context.add_widget(self._widget)

    def initialize(self):
        self.sliders()
        self.motor_switcher()


    def state_update(self, joint_states):
        if not self._initial_joints:
            self._initial_joints = joint_states
        for i in range(0, len(joint_states.name)):
            return


    def sliders(self):
        for i in range(0, len(self._initial_joints.name)):
            self._motorValues[self._initial_joints.name[i]] = self._initial_joints.position[i]

        i = 0
        for k,v in self._motorValues.items():
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
            group.moveToThread(self._widget.gridLayout_2.thread())
            self._widget.gridLayout_2.addWidget(group, i / 5, i % 5)
            i = i+1

    def motor_switcher(self):
        self._widget.treeWidget.setHeaderLabel("Motors")
        self._motorCheckBody.setText(0, "Body")
        self._motorCheckBody.setFlags(self._motorCheckBody.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckHead.setText(0, "Head")
        self._motorCheckHead.setFlags(self._motorCheckHead.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckArms.setText(0, "Arms")
        self._motorCheckArms.setFlags(self._motorCheckArms.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckLegs.setText(0, "Legs")
        self._motorCheckLegs.setFlags(self._motorCheckLegs.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckLArm.setText(0, "Left Arm")
        self._motorCheckLArm.setFlags(self._motorCheckLArm.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckRArm.setText(0, "Right Arm")
        self._motorCheckRArm.setFlags(self._motorCheckRArm.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckLLeg.setText(0, "Left Leg")
        self._motorCheckLLeg.setFlags(self._motorCheckLLeg.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self._motorCheckRLeg.setText(0, "Right Leg")
        self._motorCheckRLeg.setFlags(self._motorCheckRLeg.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)

        for k,v in self._motorValues.items():
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



    def slider_update(self):
        for k,v in self._textFields.items():
            v.setText(str(self._sliders[k].value()))

    def textfield_update(self):
        for k,v in self._sliders.items():
            try:
                v.setValue(int(self._textFields[k].text()))
            except ValueError:
                continue

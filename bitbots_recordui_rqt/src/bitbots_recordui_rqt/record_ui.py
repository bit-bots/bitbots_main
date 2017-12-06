#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospkg
import rospy
import time
import math


from copy import deepcopy
from python_qt_binding.QtCore import Qt, QMetaType, QDataStream, QVariant
from python_qt_binding import loadUi
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QTreeWidget, QTreeWidgetItem,QListWidgetItem, \
	QSlider, QGroupBox, QVBoxLayout, QLabel, QLineEdit, QListWidget, QAbstractItemView, QFileDialog, QDoubleSpinBox, QMessageBox
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

	def keyPressEvent(self, e):
		return

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
		self._selected_frame  = None

		self._currentGoals = {}                 # this is the data about the current unsaved frame
		self._currentDuration = 1.0
		self._currentPause = 0.0
		self._currentName = "new frame"

		self._workingValues = {}                # this is the data about the frame that is displayed
		self._workingDuration = 1.0
		self._workingPause = 0.0
		self._workingName = self._currentName

		self._current = True

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

		rospy.Subscriber("/joint_states", JointState, self.state_update, queue_size=1)
		self._joint_pub = rospy.Publisher("/motor_goals", JointTrajectory, queue_size=1)

		while not self._initial_joints:
			if not rospy.is_shutdown():
				time.sleep(0.5)
				print("wait")
			else:
				return

		self.initialize()

		context.add_widget(self._widget)

	def initialize(self):
		for i in range(0, len(self._initial_joints.name)):
			self._currentGoals[self._initial_joints.name[i]] = self._initial_joints.position[i]
			self._workingValues[self._initial_joints.name[i]] = self._initial_joints.position[i]
			self._motorSwitched[self._initial_joints.name[i]] = True

		self.motor_controller()
		self.motor_switcher()
		self.button_connect()
		self.box_ticked()
		self.frame_list()
		self.update_frames()
		self.set_sliders_and_text_fields(True)

	def state_update(self, joint_states):
		if not self._initial_joints:
			self._initial_joints = joint_states
			time.sleep(1)

		for i in range(0, len(joint_states.name)):
			if (not self._motorSwitched[joint_states.name[i]]):
				self._workingValues[joint_states.name[i]] = joint_states.position[i]

		self.set_sliders_and_text_fields(manual=False)

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
			slider.setMinimum(-181)
			slider.setMaximum(181)
			slider.valueChanged.connect(self.slider_update)
			self._sliders[k] = slider

			textfield = QLineEdit()
			textfield.setText('0')
			textfield.textEdited.connect(self.textfield_update)
			textfield.setValidator(QDoubleValidator(-181.0, 181.0, 2))
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
		self._widget.buttonUntilFrame.clicked.connect(self.play_until)
		self._widget.buttonGotoFrame.clicked.connect(self.goto_frame)
		self._widget.buttonGotoInit.clicked.connect(self.goto_init)
		self._widget.buttonRecord.clicked.connect(self.record)

		self._widget.buttonDuplicateFrame.clicked.connect(self.duplicate)
		self._widget.buttonDeleteFrame.clicked.connect(self.delete)

		self._widget.buttonUndo.clicked.connect(self.undo)
		self._widget.buttonRedo.clicked.connect(self.redo)

	def new(self):
			if len(self._widget.frameList) > 1:
				message = "This will discard your current Animation. Continue?"
				sure = QMessageBox.question(self._widget, 'Sure?', message, QMessageBox.Yes | QMessageBox.No)
				if sure == QMessageBox.Yes:
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
		if len(self._widget.frameList) > 1:
			message = "This will discard your current Animation. Continue?"
			sure = QMessageBox.question(self._widget, 'Sure?', message, QMessageBox.Yes | QMessageBox.No)
			if sure == QMessageBox.No:
				return
		my_file = QFileDialog.getOpenFileName()[0]
		if my_file:
			self._recorder.load_animation(my_file)
		self.update_frames()

		metadata = self._recorder.get_meta_data()

		self._widget.lineAnimationName.setText(metadata[0])
		self._widget.lineAuthor.setText(metadata[2])
		self._widget.lineVersion.setText(str(metadata[1]))
		self._widget.fieldDescription.setPlainText(metadata[3])

	def play(self):
		self._recorder.play()

	def play_until(self):
		steps = self._recorder.get_animation_state()
		for i in range(0, len(steps)):
			self._recorder.play(i)
			if steps[i]["name"] == self._selected_frame:
				return
	def goto_frame(self):
		self.set_all_joints_stiff()
		msg = JointTrajectory()
		msg.header.stamp = rospy.Time.from_seconds(time.time())
		#msg.joint_names= self._initial_joints.name
		point = JointTrajectoryPoint()
		print(self._workingValues.values())
		#point.positions= self._workingValues.values()

		for k,v in self._workingValues.items():
			msg.joint_names.append(k)
			point.positions.append(v)
		point.velocities = [30.0] * len(self._initial_joints.name)
		msg.points = [point]
		self._joint_pub.publish(msg)

	def goto_init(self):
		self.set_all_joints_stiff()
		msg = JointTrajectory()
		msg.header.stamp = rospy.Time.from_seconds(time.time()) #why not rospy.Time.now() ??
		msg.joint_names = self._initial_joints.name
		point = JointTrajectoryPoint()
		point.positions = [0] * len(self._initial_joints.name)
		point.velocities = [30.0] * len(self._initial_joints.name)
		msg.points = [point]
		self._joint_pub.publish(msg)

	def set_all_joints_stiff(self):
		for k, v in self._treeItems.items():
			v.setCheckState(0, Qt.Checked)

	def duplicate(self):
		try:
			frame = self._widget.frameList.selectedItems()[0].text()
		except:
			return
		if frame:
			self._recorder.duplicate(frame)
			self.update_frames()

	def delete(self):
		try:
			frame = self._widget.frameList.selectedItems()[0].text()
		except:
			return
		if frame:
			self._recorder.delete(frame)
			self.update_frames()

	def record(self):
		if self._widget.frameList.currentItem().text() == "#CURRENT_FRAME":

			for state in self._recorder.get_animation_state():
				if self._workingName == state["name"]:
					QMessageBox.information(self._widget, "you messed up", "frame name is already in use")
					return
			self._recorder.record(self._workingValues,
								  self._widget.lineFrameName.text(),
								  self._widget.spinBoxDuration.value(),
								  self._widget.spinBoxPause.value())
		else:
			current_row = self._widget.frameList.currentRow()
			print(current_row)
			self._recorder.record(self._workingValues,
								  self._widget.lineFrameName.text(),
								  self._widget.spinBoxDuration.value(),
								  self._widget.spinBoxPause.value(),
								  current_row,
								  True)

		self.update_frames()

	def undo(self):
		self._recorder.undo()
		self.update_frames()

	def redo(self):
		self._recorder.redo()
		self.update_frames()

	def frame_list(self):
		self._widget.frameList.itemClicked.connect(self.frame_select)
		self._widget.lineFrameName.textEdited.connect(self.frame_meta_update)
		self._widget.spinBoxPause.valueChanged.connect(self.frame_meta_update)
		self._widget.spinBoxDuration.valueChanged.connect(self.frame_meta_update)

	def frame_meta_update(self):
		self._workingDuration = self._widget.spinBoxDuration.value()
		self._workingPause = self._widget.spinBoxPause.value()
		self._workingName = self._widget.lineFrameName.text()

	def frame_select(self):
		selected_frame_name = self._widget.frameList.currentItem().text()
		self._selected_frame = None


		for v in self._recorder.get_animation_state():
			if v["name"] == selected_frame_name:
				self._selected_frame = v
				break

		#save current values to _currentValues if switching from current frame to different one
		#when switching from current to different, save current values


		if selected_frame_name == "#CURRENT_FRAME":
			self._workingValues = deepcopy(self._currentGoals)
			self._workingName = deepcopy(self._currentName)
			self._workingDuration = deepcopy(self._currentDuration)
			self._workingPause = deepcopy(self._currentPause)

			self._current = True
		else:
			if self._current:
				self._currentGoals = deepcopy(self._workingValues)
				self._currentName = deepcopy(self._workingName)
				self._currentDuration = deepcopy(self._workingDuration)
				self._currentPause = deepcopy(self._workingPause)

			self._workingValues = self._selected_frame["goals"]
			self._workingName = self._selected_frame["name"]
			self._workingPause = self._selected_frame["pause"]
			self._workingDuration = float(self._selected_frame["duration"])

			self._current = False

		self.set_sliders_and_text_fields(manual=True)

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
			self._workingValues[k] = math.radians(v.value())
			if self._workingValues[k] < -math.pi:
				self._workingValues[k] = -math.pi
			elif self._workingValues[k] > math.pi:
				self._workingValues[k] = math.pi
		self.set_sliders_and_text_fields(manual=True)

	def textfield_update(self):
		for k, v in self._textFields.items():
			try:
				self._workingValues[k] = math.radians(float(v.text()))
			except ValueError:
				continue
		self.set_sliders_and_text_fields(manual=True)

	def set_sliders_and_text_fields(self, manual):
		"""
		Updates the text fields and sliders ins self._sliders and self._textfields and also frame name and duration and pause 
		to the values in self._workingValues. If one of the values is greater than pi( or lesser than negative pi), all loaded values
		can't be radians and thus are interpreted as degrees.
		:return: 
		"""
		deg = False
		for k, v in self._workingValues.items():
				if v > math.pi or v < -math.pi:
					deg = True

		for k, v in self._workingValues.items():
			try:
				if not self._motorSwitched[k] or manual:
					if deg == False:
						self._textFields[k].setText(str(int(math.degrees(v))))
						self._sliders[k].setValue(int(math.degrees(v)))
					else:
						self._textFields[k].setText(str(int(v)))
						self._sliders[k].setValue(int(v))
			except KeyError:
				continue
		if manual:
			self._widget.lineFrameName.setText(self._workingName)
			self._widget.spinBoxDuration.setValue(self._workingDuration)
			self._widget.spinBoxPause.setValue(float(self._workingPause))

	def box_ticked(self):
		msg = JointTrajectory()
		msg.header.stamp = rospy.Time.now()
		msg.joint_names = []
		msg.points = []
		msg.points.append(JointTrajectoryPoint())

		for k, v in self._treeItems.items():
			if self._motorSwitched[k] != (v.checkState(0) == Qt.Checked): #if motor stiffness should change
				msg.joint_names.append(k)
				msg.points[0].positions.append(self._workingValues[k])
				if v.checkState(0) == Qt.Checked:
					msg.points[0].effort.append(1.0)
				else:
					msg.points[0].effort.append(0.0)

				msg.points[0].velocities.append(30.0)

			self._motorSwitched[k] = (v.checkState(0) == Qt.Checked)

		if len(msg.joint_names) > 0:
			self._joint_pub.publish(msg)

		for k, v in self._motorSwitched.items():
			self._textFields[k].setEnabled(v)
			self._sliders[k].setEnabled(v)
			print(k + " " + str(v))

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
		self._widget.frameList.addItem(current)
		self._widget.frameList.setCurrentItem(current)
		self._current = True

	def change_frame_order(self, new_order):
		""" Calls the recorder to update frame order and updates the gui"""
		self._recorder.change_frame_order(new_order)
		self.update_frames()

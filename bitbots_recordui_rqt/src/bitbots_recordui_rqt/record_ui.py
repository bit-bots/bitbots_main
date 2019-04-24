#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospkg
import rospy
import time
import math
import inspect

from copy import deepcopy
from python_qt_binding.QtCore import Qt, QMetaType, QDataStream, QVariant, pyqtSignal
from python_qt_binding import loadUi
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QMainWindow, QTreeWidget, QTreeWidgetItem,QListWidgetItem, \
    QSlider, QGroupBox, QVBoxLayout, QLabel, QLineEdit, QListWidget, QAbstractItemView, QFileDialog, QDoubleSpinBox, QMessageBox, \
    QInputDialog, QShortcut
from python_qt_binding.QtGui import QDoubleValidator, QKeySequence

from bitbots_msgs.msg import JointCommand, JointTorque
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import os

from .animation_recording import Recorder



class DragDropList(QListWidget):
    """ QListWidget with an event that is called when a drag and drop action was performed."""
    keyPressed = pyqtSignal()

    def __init__(self, parent, ui):
        super(DragDropList, self).__init__(parent)

        self.ui = ui
        self.setAcceptDrops(True)


    def dropEvent(self, e):
        super(DragDropList, self).dropEvent(e)
        items = []
        for i in range(0, self.count()):
            items.append(self.item(i).text())
        self.ui.change_frame_order(items)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Delete:
            super(DragDropList, self).keyPressEvent(event)
            self.keyPressed.emit()
        elif event.key() == Qt.Key_Up and self.currentRow()-1 >= 0:
                self.setCurrentRow(self.currentRow()-1)
        elif event.key() == Qt.Key_Down and self.currentRow()+1 < self.count():
            self.setCurrentRow(self.currentRow()+1)




class RecordUI(Plugin):
    ''' class containing the UI part of the framework'''
    def __init__(self, context):
        super(RecordUI, self).__init__(context)


        self._widget = QMainWindow()
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

        self._robot_anim_path = None

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

        #saves configuration of the trees checkboxes for each of the tree modes.
        self._checkBoxesSave = {}
        self._checkBoxesPower = {}
        self._previousTreeMode = 0

        self.setObjectName('Record Animation')

        self._initial_joints = None

        self._widget.frameList = DragDropList(self._widget, self)
        self._widget.verticalLayout_2.insertWidget(0, self._widget.frameList)
        self._widget.frameList.setDragDropMode(QAbstractItemView.InternalMove)

        self.state_sub = rospy.Subscriber("/joint_states", JointState, self.state_update, queue_size=1, tcp_nodelay=True)
        self._joint_pub = rospy.Publisher("/record_motor_goals", JointCommand, queue_size=1)
        self.effort_pub = rospy.Publisher("/ros_control/set_torque_individual", JointTorque, queue_size=1)

        self.ids = {"HeadPan": 0,
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
               "RAnkleRoll": 19}

        self._initial_joints = JointState()

        self.update_time = rospy.Duration(0.1)

        for k,v in self.ids.iteritems():
            print(k)
            self._initial_joints.name.append(k)
            self._initial_joints.position.append(0)

        while not self._initial_joints:
            if not rospy.is_shutdown():
                time.rospy.sleep(0.5)
                print("wait")
            else:
                return

        self.initialize()

        context.add_widget(self._widget)
        self._widget.statusBar.showMessage("Initialization complete.")

    def initialize(self):
        for i in range(0, len(self._initial_joints.name)):
            self._currentGoals[self._initial_joints.name[i]] = self._initial_joints.position[i]
            self._workingValues[self._initial_joints.name[i]] = self._initial_joints.position[i]
            self._motorSwitched[self._initial_joints.name[i]] = True


        #sets up a query on which robot is used, selects animation path according to this    
        items = ["Amy", "Rory", "Clara", "Danny", "other"]
        item, button_pressed= QInputDialog.getItem(self._widget, "Animation Path required", \
            "Please select the robot you are using or 'other' to enter the path where animations are stored manually.", items, 0, False)
        if button_pressed == False:
            self._robot_anim_path = ""
        if button_pressed:
            print("Robot selected: "+item)
            if item == "Amy":
                self._robot_anim_path = "bitbots@nuc1:~/wolfgang_ws/src/wolfgang_robot/wolfgang_animations/animations/motion/"
            if item == "Rory":
                self._robot_anim_path = "bitbots@nuc2:~/wolfgang_ws/src/wolfgang_robot/wolfgang_animations/animations/motion/"
            if item == "Clara":
                self._robot_anim_path = "bitbots@odroid3:~/minibot_ws/src/bitbots_minibot/minibot_animations/animations/motion/"
            if item == "Danny":
                self._robot_anim_path = "bitbots@odroid4:~/minibot_ws/src/bitbots_minibot/minibot_animations/animations/motion/"
            if item == "other":
                self._robot_anim_path, button_pressed = QInputDialog.getText(self._widget, "Animation Path required", \
                    "Please enter a different path.", QLineEdit.Normal, "")
                if button_pressed == False or self._robot_anim_path == "":
                    self._robot_anim_path = ""
                
        print("Set animation path to: "+str(self._robot_anim_path+"record.json"))

        initTorque = {}
        for k, v in self._workingValues.items():
            initTorque[k] = 2

        self._checkBoxesPower['#CURRENT_FRAME'] = initTorque

        self.motor_controller()
        self.motor_switcher()
        self.action_connect()
        self.box_ticked()
        self.frame_list()
        self.update_frames()
        self.set_sliders_and_text_fields(manual=True)

    def state_update(self, joint_states):
        if not self._initial_joints:
            self._initial_joints = joint_states
            time.rospy.sleep(1)
        else:
            for i in range(0, len(joint_states.name)):
                if (not self._motorSwitched[joint_states.name[i]]):
                    self._workingValues[joint_states.name[i]] = joint_states.position[i]


        rospy.sleep(self.update_time)
        self.set_sliders_and_text_fields(manual=False)


    def motor_controller(self):
        """
        Sets up the GUI in the middle of the Screen to control the motors. 
        Uses self._motorValues to determine which motors are present.
        """
        i = 0
        for k, v in sorted(self._currentGoals.items()):
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
            #textfield.setValidator(QDoubleValidator(-181.0, 181.0, 2))
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
        self._widget.actionLeft.triggered.connect(lambda: self.mirrorFrame("L"))
        self._widget.actionRight.triggered.connect(lambda: self.mirrorFrame("R"))
        self._widget.actionInvert.triggered.connect(self.invertFrame)
        self._widget.actionUndo.triggered.connect(self.undo)
        self._widget.actionRedo.triggered.connect(self.redo)
        self._widget.actionHelp.triggered.connect(self.help)

        self._widget.buttonRecord.clicked.connect(self.record)
        self._widget.buttonRecord.shortcut = QShortcut(QKeySequence("Space"), self._widget)
        self._widget.buttonRecord.shortcut.activated.connect(self.record)


        self._widget.frameList.keyPressed.connect(self.delete)

        self._widget.treeModeSelector.currentIndexChanged.connect(self.treeModeChanged)

    def help(self):
        helpDialog = QMessageBox.about(self._widget, "About RecordUI", "This is RecordUI, a tool to record robot animations.\n \n Keyboard shortcuts: \n \n \
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
            Invert: Ctrl + Down Arrow")

    def new(self):
        '''
        Deletes all currently recorded frames
        '''
        if len(self._widget.frameList) > 1:
            message = "This will discard your current Animation. Continue?"
            sure = QMessageBox.question(self._widget, 'Sure?', message, QMessageBox.Yes | QMessageBox.No)
            if sure == QMessageBox.Yes:
                self._recorder.clear()
                self.update_frames()

    def save(self):
        '''
        Saves all currently recorded frames into a json file
        '''
        self.treeModeChanged(self._previousTreeMode)
        self.set_metadata()
        if not self._saveDir:
            self._saveDir = QFileDialog.getExistingDirectory()
        status = self._recorder.save_animation(self._saveDir, self._widget.lineAnimationName.text(), self._checkBoxesSave)
        self._widget.statusBar.showMessage(status)

    def save_as(self):
        '''
        Saves all currently recorded frames into a json file, which is saved at a user specified location
        '''
        self.treeModeChanged(self._previousTreeMode)
        self._saveDir = QFileDialog.getExistingDirectory()
        self._recorder.save_animation(self._saveDir, self._widget.lineAnimationName.text(), self._checkBoxesSave)

    def set_metadata(self):
        status = self._recorder.set_meta_data(self._widget.lineAnimationName.text(),
                                     self._widget.lineVersion.text(),
                                     self._widget.lineAuthor.text(),
                                     self._widget.fieldDescription.toPlainText())
        self._widget.statusBar.showMessage(status)
    def open(self):
        '''
        Deletes all current frames and instead loads an animation from a json file
        '''
        if len(self._widget.frameList) > 1:
            message = "This will discard your current Animation. Continue?"
            sure = QMessageBox.question(self._widget, 'Sure?', message, QMessageBox.Yes | QMessageBox.No)
            if sure == QMessageBox.No:
                return
        my_file = QFileDialog.getOpenFileName()[0]
        if my_file:
            status = self._recorder.load_animation(my_file)
            if status == "":
                status = "Load successful."
            self._widget.statusBar.showMessage(status)

        animstate = self._recorder.get_animation_state()
        for i in animstate:
            self._checkBoxesPower[i['name']] = i['torque']



        self.update_frames()

        metadata = self._recorder.get_meta_data()

        self._widget.lineAnimationName.setText(metadata[0])
        self._widget.lineAuthor.setText(metadata[2])
        self._widget.lineVersion.setText(str(metadata[1]))
        self._widget.fieldDescription.setPlainText(metadata[3])

    def play(self):
        '''
        Plays the animation
        '''
        status = self._recorder.play(self._robot_anim_path)
        self._widget.statusBar.showMessage(status)

    def play_until(self):
        '''
        Plays the animation up to a certain frame
        '''
        steps = self._recorder.get_animation_state()
        j = 0
        for i in range(0, len(steps)):
            j += 1
            print(steps[i]["name"])
            if steps[i]["name"] == self._selected_frame["name"]:
                break
        self._recorder.play(self._robot_anim_path, j)

    def goto_frame(self):
        '''
        Plays one single frame
        '''
        self.set_all_joints_stiff()

        pos_msg = JointCommand()
        pos_msg.joint_names = []
        pos_msg.velocities = [1.0] * 20
        pos_msg.positions = []
        pos_msg.accelerations = [-1.0] * 20
        pos_msg.max_currents = [-1.0] * 20

        for k,v in self._workingValues.items():
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
            self._widget.frameList.setCurrentRow(self._widget.frameList.currentRow()+1)
            self.goto_frame()

    def goto_init(self):
        '''
        Plays init frame, sets all motor values to 0
        '''
        self.set_all_joints_stiff()

        if self._widget.frameList.currentItem().text() == "#CURRENT_FRAME":
            for k, v in self._workingValues.iteritems():
                self._workingValues[k] = 0.0
        for k, v in self._currentGoals.iteritems():
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
        if self._widget.frameList.currentItem().text() == '#CURRENT_FRAME':
            for k, v in self._treeItems.items():
                v.setCheckState(0, Qt.Checked)

        torque_msg = JointTorque()
        torque_msg.joint_names = []
        torque_msg.on = []

        for k, v in sorted(self._treeItems.items()):
            torque_msg.joint_names.append(k)
            torque_msg.on.append(True)

        self.effort_pub.publish(torque_msg)

    def duplicate(self):
        '''
        Creates a copy of the selected frame
        '''
        try:
            frame = self._widget.frameList.selectedItems()[0].text()
        except:
            return
        if frame:
            self._recorder.duplicate(frame)
            self.update_frames()

    def delete(self):
        '''
        Deletes a frame
        '''
        try:
            frame = self._widget.frameList.selectedItems()[0].text()
        except:
            return
        if frame:
            self._recorder.delete(frame)
            self.update_frames()

    def record(self, keep=False):
        '''
        Records a frame
        '''
        if not self._widget.frameList.currentItem() == None:
            if self._widget.frameList.currentItem().text() == "#CURRENT_FRAME":
                x = True
                n = 0
                for state in self._recorder.get_animation_state():
                    if self._workingName == state["name"]:
                        while(x):
                            x = False
                            for state in self._recorder.get_animation_state():
                                if self._workingName+str(n) == state["name"]:
                                    n+=1
                                    x = True
                        self._workingName = self._workingName+str(n)

                self.set_sliders_and_text_fields(manual=True)

                self._checkBoxesPower[self._workingName] = {}
                initTorque = {}
                for k, v in self._workingValues.items():
                    initTorque[k] = 2

                self._checkBoxesPower[self._workingName] = initTorque

                self._recorder.record(self._workingValues,
                                      initTorque,
                                      self._widget.lineFrameName.text(),
                                      self._widget.spinBoxDuration.value(),
                                      self._widget.spinBoxPause.value())
                self._currentGoals = deepcopy(self._workingValues)
            else:
                current_row = self._widget.frameList.currentRow()

                self._recorder.record(self._workingValues,
                                      self._checkBoxesPower[self._widget.frameList.currentItem().text()],
                                      self._widget.lineFrameName.text(),
                                      self._widget.spinBoxDuration.value(),
                                      self._widget.spinBoxPause.value(),
                                      current_row,
                                      True)

 

        self.update_frames(keep)

    def undo(self):
        status = self._recorder.undo()
        self._widget.statusBar.showMessage(status)
        self.update_frames()

    def redo(self):
        status = self._recorder.redo()
        self._widget.statusBar.showMessage(status)
        self.update_frames()

    def mirrorFrame(self, direction):
        '''
        Copies all motor values from one side of the robot to the other. Inverts values, if necessary
        '''
        opposite = "L"
        if direction == "L":
            opposite = "R"
        for k, v in self._workingValues.items():
            if k[0] == opposite:
                for k1, v1 in self._workingValues.items():
                    if k1 == str(direction)+k[1:]:
                        self._workingValues[k1] = v * -1

        boxmode = 0
        if self._widget.treeModeSelector.currentIndex() == 0:
            boxmode = self._checkBoxesPower[self._widget.frameList.currentItem().text()]
        elif self._widget.treeModeSelector.currentIndex() == 1:
            boxmode = self._checkBoxesSave[self._widget.frameList.currentItem().text()]

        for k, v in boxmode.items():
            if k[0] == opposite:
                for k1, v1 in boxmode.items():
                    if k1 == str(direction)+k[1:]:
                        if boxmode[k] == 0:
                            boxmode[k1] = 0
                        elif boxmode[k] ==2:
                            boxmode[k1] = 2

        self.updateTreeConfig(self._widget.treeModeSelector.currentIndex())
        self.box_ticked()
        

    def invertFrame(self):
        '''
        Copies all values from the left side to the right and all values from the right side to the left. 
        Inverts values, if necessary
        '''
        tempDict={}
        for k, v in self._workingValues.items():
            if k[0] == "L":
                tempDict["R"+k[1:]] = -v
            elif k[0] == "R":
                tempDict["L"+k[1:]] = -v
        for k, v in tempDict.items():
            self._workingValues[k] = v

        boxmode = 0
        if self._widget.treeModeSelector.currentIndex() == 0:
            boxmode = self._checkBoxesPower
        elif self._widget.treeModeSelector.currentIndex() == 1:
            boxmode = self._checkBoxesSave

        for k, v in boxmode[self._widget.frameList.currentItem().text()].items():

            if k[0] == "L":
                if v == 2:
                    tempDict["R"+k[1:]] = 2
                elif v == 0:
                    tempDict["R"+k[1:]] = 0
            elif k[0] == "R":
                if v == 2:
                    tempDict["L"+k[1:]] = 2
                elif v == 0:
                    tempDict["L"+k[1:]] = 0

        boxmode[self._widget.frameList.currentItem().text()] = deepcopy(tempDict)
        self.updateTreeConfig(self._widget.treeModeSelector.currentIndex())
        self.box_ticked()

    def frame_list(self):
        #self._widget.frameList.itemClicked.connect(self.frame_select)
        self._widget.frameList.itemSelectionChanged.connect(self.frame_select)
        self._widget.lineFrameName.textEdited.connect(self.frame_meta_update)
        self._widget.spinBoxPause.valueChanged.connect(self.frame_meta_update)
        self._widget.spinBoxDuration.valueChanged.connect(self.frame_meta_update)

    def frame_meta_update(self):
        self._workingDuration = self._widget.spinBoxDuration.value()
        self._workingPause = self._widget.spinBoxPause.value()
        self._workingName = self._widget.lineFrameName.text()

    def frame_select(self):
        if not (self._widget.frameList.currentItem() == None):
            self.copyOldTreeConfig()
            selected_frame_name = self._widget.frameList.currentItem().text()
            self._selected_frame = None



            for v in self._recorder.get_animation_state():
                if v["name"] == selected_frame_name:
                    self._selected_frame = v
                    break

        #save current values to _currentValues if switching from current frame to different one
        #when switching from current to different, save current values


            if selected_frame_name == "#CURRENT_FRAME":
                self._widget.treeModeSelector.setCurrentIndex(0)
                self.updateTreeConfig(0)
                self._widget.treeModeSelector.setEnabled(False)
                self._workingValues = deepcopy(self._currentGoals)
                self._workingName = deepcopy(self._currentName)
                self._workingDuration = deepcopy(self._currentDuration)
                self._workingPause = deepcopy(self._currentPause)

                self._current = True
            else:
                if self._selected_frame == None:
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
                self.updateTreeConfig(self._previousTreeMode)
            
            if  self._widget.treeModeSelector.currentIndex() == 0:
                for k, v in self._treeItems.items():
                    self._motorSwitched[k] = (v.checkState(0) == 2)


            for k, v in self._motorSwitched.items():
                self._textFields[k].setEnabled(v)
                self._sliders[k].setEnabled(v)

        self.set_sliders_and_text_fields(manual=False)
            #self.set_sliders_and_text_fields(manual=True)

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

        self._widget.motorTree.itemClicked.connect(self.box_ticked)

    def copyOldTreeConfig(self):
        '''
        Saves the current configuration of the motor tree checkboxes into the corresponding dictionary
        '''
        if not self._selected_frame == None:
            tempDict={}
            for k, v in self._treeItems.items():
                tempDict[k]= self._treeItems[k].checkState(0)

            if self._previousTreeMode == 1:
                self._checkBoxesSave[self._selected_frame["name"]] = deepcopy(tempDict)
            elif self._previousTreeMode == 0:
                self._checkBoxesPower[self._selected_frame["name"]] = deepcopy(tempDict)

    def updateTreeConfig(self, index):
        '''
        Loads the new configuration of the motor tree depending on the change
        '''
        tempDict2={}
        if not self._widget.frameList.currentItem() == None:
            if not self._widget.frameList.currentItem().text() == "#CURRENT_FRAME":
                if index == 1:
                    if self._selected_frame["name"] in self._checkBoxesSave.keys():
                        tempDict2 = deepcopy(self._checkBoxesSave[self._selected_frame["name"]])
                    else:
                        for k in self._workingValues:
                            tempDict2[k] = 2
                    #rospy.loginfo('Selected Save mode.')
                elif index == 0:
                    if self._selected_frame["name"] in self._checkBoxesPower.keys():
                        tempDict2 = deepcopy(self._checkBoxesPower[self._selected_frame["name"]])
                    else:
                        for k in self._workingValues:
                            tempDict2[k] = 2
                    #rospy.loginfo('Selected Power mode.')
            else:
                for k in self._workingValues:
                    tempDict2[k] = 2
            self._previousTreeMode = index
            for k, v in tempDict2.items():
                if v == 0:
                    self._treeItems[k].setCheckState(0, Qt.Unchecked)
                elif v == 1:
                    self._treeItems[k].setCheckState(0, Qt.PartiallyChecked)
                elif v == 2:
                    self._treeItems[k].setCheckState(0, Qt.Checked)

    def treeModeChanged(self, index):
        self.copyOldTreeConfig()
        self.updateTreeConfig(index)


    def slider_update(self):
        '''
        Updates all sliders, checks whether a value is too large, then replaces it
        '''
        for k, v in self._sliders.items():
            self._workingValues[k] = math.radians(v.value())
            if self._workingValues[k] < -math.pi:
                self._workingValues[k] = -math.pi
            elif self._workingValues[k] > math.pi:
                self._workingValues[k] = math.pi
        self.set_sliders_and_text_fields(manual=True)

    def textfield_update(self):
        '''
        Updates all textfields.
        '''
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
                    #if not self._motorSwitched[k] or manual:
                        self._textFields[k].setText(str(int(round(math.degrees(v)))))
                        self._sliders[k].setValue(int(round(math.degrees(v))))

            except KeyError:
                rospy.logwarn("Found a key-value pair for motor (%s), which doesn't seem to exist (yet). Ignoring it." % k)
                self._widget.statusBar.showMessage("Found a key-value pair for motor (%s), which doesn't seem to exist (yet). Ignoring it.")
                continue
            except RuntimeError:
                rospy.logwarn("Tried to access a PyQt element that no longer exists. This happens when you close the framework.")
                self._widget.statusBar.showMessage("Tried to access a PyQt element that no longer exists. This happens when you close the framework.")
                break
        if manual:
            self._widget.lineFrameName.setText(self._workingName)
            self._widget.spinBoxDuration.setValue(self._workingDuration)
            self._widget.spinBoxPause.setValue(self._workingPause)

    def box_ticked(self):
        
        '''
        Controls whether a checkbox has been clicked, and reacts.
        '''
        for k, v in self._treeItems.items():
            self._motorSwitched[k] = (v.checkState(0) == 2)


        for k, v in self._motorSwitched.items():
            self._textFields[k].setEnabled(v)
            self._sliders[k].setEnabled(v)

        self.set_sliders_and_text_fields(manual=False)

        ## Wolfgang part

        torque_msg = JointTorque()
        torque_msg.joint_names = []
        torque_msg.on = []

        if  self._widget.treeModeSelector.currentIndex() == 0:
            for k, v in sorted(self._treeItems.items()):
                torque_msg.joint_names.append(k)
                torque_msg.on.append(v.checkState(0) == 2)


        pos_msg = JointCommand()
        pos_msg.joint_names = []
        pos_msg.velocities = [1.0] * 20
        pos_msg.positions = []
        pos_msg.accelerations = [-1.0] * 20
        pos_msg.max_currents = [-1.0] * 20

        for k,v in self._workingValues.items():
            pos_msg.joint_names.append(k)
            pos_msg.positions.append(v)

        self._joint_pub.publish(pos_msg)
        self.effort_pub.publish(torque_msg)
        if not self._widget.frameList.currentItem() == None:
            if not self._widget.frameList.currentItem().text() == "#CURRENT_FRAME":
                self.treeModeChanged(self._widget.treeModeSelector.currentIndex())
                self.record(keep=True)

    def update_frames(self, keep=False):
        
        """
        updates the list of frames present in the current animation
        :return: 
        """
        current_index = self._widget.frameList.currentIndex()
        current_mode = self._widget.treeModeSelector.currentIndex()
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
        if keep:    
            self._widget.frameList.setCurrentIndex(current_index)
            self._widget.treeModeSelector.setCurrentIndex(current_mode)
        else:
            self._widget.frameList.setCurrentItem(current)
            self._current = True

    def change_frame_order(self, new_order):
        """ Calls the recorder to update frame order and updates the gui"""
        self._recorder.change_frame_order(new_order)
        self.update_frames()

    def shutdown_plugin(self):
        """Clean up by sending the HCM a signal that we are no longer recording and by stopping publishers"""
        self._joint_pub.publish(JointCommand())
        rospy.sleep(1)
        #self.state_sub.unregister()
        #self._joint_pub.unregister()
        #self.effort_pub.unregister()
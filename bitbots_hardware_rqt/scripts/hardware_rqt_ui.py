#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#TODO: foot sensors
#segfault imu
#timestamp
#logging
#cleanup

from python_qt_binding.QtWidgets import QMainWindow, QLabel, QTableWidgetItem, QTableWidget,QComboBox, QInputDialog, QLineEdit
import pyqtgraph as pg
from python_qt_binding import loadUi, QtCore, QtGui
from rqt_gui_py.plugin import Plugin

from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState, Imu
from bitbots_buttons.msg import Buttons
from humanoid_league_msgs.msg import RobotControlState

from std_msgs.msg import String

import rospy, rospkg
import numpy as np
import os, math, random
import threading
import json
import getpass, datetime, sys, socket

class HardwareUI(Plugin):
    """Backend for the GUI. Receives UDP data and distributes it across all GUI elements."""
    tetrigger = QtCore.pyqtSignal()
    totrigger = QtCore.pyqtSignal()
    votrigger = QtCore.pyqtSignal()
    imutrigger = QtCore.pyqtSignal(Imu)

    def __init__(self, context):
        """Loads up the master.ui GUI file and adds some additional widgets that cannot be added beforehand. Also sets some necessary variables."""
        super(HardwareUI, self).__init__(context)
        self._widget = QMainWindow()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('bitbots_hardware_rqt'), 'resource', 'master.ui')
        loadUi(ui_file, self._widget, {})

        self.current_tab = 0
        self.current_outer_tab = 0

        self.historydict = []
        self.colorlist = []     #Holds information about the graph colors, so we can distinguish between the different motors
        self.motornames = ['HeadPan',
                            'HeadTilt',
                            'LAnklePitch',
                            'LAnkleRoll',
                            'LElbow',
                            'LHipPitch',
                            'LHipRoll',
                            'LHipYaw',
                            'LKnee',
                            'LShoulderPitch',
                            'LShoulderRoll',
                            'RAnklePitch',
                            'RAnkleRoll',
                            'RElbow',
                            'RHipPitch',
                            'RHipRoll',
                            'RHipYaw',
                            'RKnee',
                            'RShoulderPitch',
                            'RShoulderRoll']    #We need the motor names in alphabetical order for some gui elements
        for i in range(0, 20):
            self.colorlist.append((random.randrange(50,255),random.randrange(0,255),random.randrange(0,255)))

        self.templist = [[] for i in range(20)]
        self.torquelist = [[] for i in range(20)]
        self.voltagelist = [[] for i in range(20)]

        self.crosshair = QLabel()
        self.pixmap = QtGui.QPixmap(os.path.join(rp.get_path('bitbots_hardware_rqt'), 'resource', 'cross.png'))
        self.crosshair.setPixmap(self.pixmap)
        self.crosshair.setParent(self._widget.label_4)
        self.crosshair.move(187, 187)

        self.crosshair2 = QLabel()
        self.pixmap = QtGui.QPixmap(os.path.join(rp.get_path('bitbots_hardware_rqt'), 'resource', 'cross.png'))
        self.crosshair2.setPixmap(self.pixmap)
        self.crosshair2.setParent(self._widget.label_11)
        self.crosshair2.move(87, 87)

        self.make_Graphs()

        self.votrigger.connect(lambda: self.update_graph(self.voltageplt, self.voltagelist, self.combobox2, 3))
        self.tetrigger.connect(lambda: self.update_graph(self.tempplt, self.templist, self.combobox3, 1))
        self.totrigger.connect(lambda: self.update_graph(self.torqueplt, self.torquelist, self.combobox, 2))


        self.diagnostics = rospy.Subscriber("/diagnostics", DiagnosticArray, self.set_motor_diagnostics)
        self.joint_states = rospy.Subscriber("/joint_states", JointState, self.set_motor_joint_states)
        self.buttons = rospy.Subscriber("/buttons", Buttons, self.set_buttons)
        self.imu = rospy.Subscriber("/imu/data", Imu, self.emit_imu_trigger)
        self.state = rospy.Subscriber("/robot_state", RobotControlState, self.set_robot_state)

        self.imutrigger.connect(self.set_imu)

        #self._robot_ip = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]
        self._robot_port, button_pressed = QInputDialog.getText(self._widget, "Change Port?", \
                "Please select the port you want to listen on. Leave blank for port 5005.", QLineEdit.Normal, "")
        if button_pressed and self._robot_port == "":
            self._robot_port = "5005"

        #self.rcvthread = ReceiverThread(self._robot_ip, self._robot_port)
        #self.rcvthread.start()


        self.write_log = False

        self._widget.GraphenView.currentChanged.connect(self.current_graph_changed)
        self._widget.tabWidget.currentChanged.connect(self.current_outer_tab_changed)


        self._widget.checkBox_5.stateChanged.connect(self.write_log_trigger)


        #self._widget.label_14.setText("IP: "+ [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0])

        context.add_widget(self._widget)
        self._widget.show()



    def calcPosInCicle(self, radOrienation):
        """return a position on the circle in the GUI"""
        yPos = -math.cos(radOrienation) * 100 + 87
        xPos = math.sin(radOrienation) * 100 + 87
        return xPos, yPos

    def write_log_trigger(self):
        """Enables or disables the output of a log file

        When triggered writes a file with a header and sets a flag to write data into the file.
        """
        if self._widget.checkBox_5.isChecked():
            filename = 'hardware_log_' + str(rospy.get_rostime().secs) + '.txt'
            self.logfile = open(filename, 'w+')
            self.logfile.write(" Host: " + os.uname()[1] + "\n User: " + getpass.getuser() + "\n Time: " + str(datetime.datetime.now()) + \
                "\n =========================\n \n")
            self.write_log = True
        elif self.write_log:
            self.write_log = False
            self.logfile.close()

    def current_graph_changed(self, change):
        """Handles the changing between tabs, so that only the active tab is updated, to reduce lag"""
        self.current_tab = change

    def current_outer_tab_changed(self, change):
        self.current_outer_tab = change

    def process_data(self, data):
        """Triggered each time we receive an UDP message. Calls methods to update each GUI element"""

        #Writes into the logfile, if logging is enabled
        if self.write_log:  #TODO: Move Logging or remove
            self.logfile.write(data)
            self.logfile.write("\n \n")

    def emit_imu_trigger(self, data):
        self.imutrigger.emit(data)

    def set_imu(self, data):
        """Updates the IMU/Gyro widgets."""
        if self.current_outer_tab == 2:
            self._widget.label_8.setText(str(math.degrees(data.angular_velocity.x)))
            self._widget.label_9.setText(str(math.degrees(data.angular_velocity.y)))
            self._widget.label_10.setText(str(math.degrees(data.angular_velocity.z)))

            self.crosshair.move((187+(math.degrees(data.angular_velocity.x)*400/360)), \
                (187+(math.degrees(data.angular_velocity.y)*400/360)))    #To place the crosshair on the circle, circle diametre is 400px, crosshair is 25px wide
            yawx, yawy = self.calcPosInCicle(data.angular_velocity.z)
            self.crosshair2.move(yawx, yawy)

    def set_buttons(self, data):
        """Updates the widgets placed in the topbar"""
        self._widget.checkBox.setCheckState(data.button1)
        self._widget.checkBox.setCheckState(data.button2)
        #timestamp = data['timestamp'].split('.')  TODO: reimplement timestamp
        #self._widget.label_13.setText(timestamp[0])

    def set_robot_state(self, data):
        self._widget.RobotState.display(data.state)

    def set_cpu_tab(self, data):
        """Updates the widgets in the CPU tab"""
        for i in data['cpulist']:
            if i['name'] == 'cpu_name':
                self._widget.tableWidget_3.setItem(0, 0, QTableWidgetItem(i['usage']))      #TODO: Configure this for robots.
                self._widget.tableWidget_3.setItem(1, 0, QTableWidgetItem(i['temp']))
            elif i['name'] == 'cpu_name2':
                self._widget.tableWidget_4.setItem(0, 0, QTableWidgetItem(i['usage']))
                self._widget.tableWidget_4.setItem(1, 0, QTableWidgetItem(i['temp']))

    def set_motor_diagnostics(self, data):
        """Updates the table in the motor overview tab"""
        self.motorheaderlist = []
        for i in range(len(data.status)):
            if data.status[i].level == 1:
                self.templist[int(data.status[i].hardware_id)-101].append(float(data.status[i].values[3].value))
                self.voltagelist[int(data.status[i].hardware_id)-101].append(float(data.status[i].values[1].value))
                if len(self.templist[i]) > 20:
                    self.templist[i].pop(0)
                if len(self.voltagelist[i]) > 20:
                    self.voltagelist[i].pop(0)    
                if self.current_tab == 0:
                    if data.status[i].level == 1:
                        if not data.status[i].values[2].value in self.motorheaderlist:
                            self.motorheaderlist.append(data.status[i].values[2].value)
                        for j in range(0, len(self.motorheaderlist)):
                            if j < 10:
                                selected = self._widget.tableWidget
                                k=j
                            else:
                                selected = self._widget.tableWidget_2
                                k=j-10

                            if self.motorheaderlist[j] == data.status[i].values[2].value:
                                selected.setItem(0, k, QTableWidgetItem(data.status[i].message))
                                selected.setItem(1, k, QTableWidgetItem(str(round(float(data.status[i].values[3].value),2))))
                                selected.setItem(3, k, QTableWidgetItem(str(round(float(data.status[i].values[1].value),2))))
                                selected.setItem(4, k, QTableWidgetItem(data.status[i].values[0].value))

            self._widget.tableWidget.setHorizontalHeaderLabels(self.motorheaderlist)
            self._widget.tableWidget_2.setHorizontalHeaderLabels(self.motorheaderlist[10:])
            self._widget.tableWidget.setEditTriggers(QTableWidget.NoEditTriggers)

        self.tetrigger.emit()
        self.votrigger.emit()

    def set_motor_joint_states(self, data):
        if self.current_tab == 0:
            self.motorheaderlist = []
            for i in range(0, len(data.name)):
                self.torquelist[i].append(float(data.effort[i]))
                if not data.name[i] in self.motorheaderlist:
                    self.motorheaderlist.append(data.name[i])
                for j in range(0, len(self.motorheaderlist)):    
                    if j < 10:
                        selected = self._widget.tableWidget
                        k=j
                    else:
                        selected = self._widget.tableWidget_2
                        k=j-10
                    for j in self.motorheaderlist:
                        if j == data.name[i]:
                            selected.setItem(2, k, QTableWidgetItem(str(round(float(math.degrees(data.position[i])),2))))
                            selected.setItem(5, k, QTableWidgetItem(str(round(float(data.effort[i]),2))))


            self._widget.tableWidget.setHorizontalHeaderLabels(self.motorheaderlist)
            self._widget.tableWidget_2.setHorizontalHeaderLabels(self.motorheaderlist[10:])

            self.totrigger.emit()

    def update_graph(self, graph, glist, cbox, number):
        """Updates the graph that displays the motors temperature"""
        if self.current_tab == number:
            graph.clear()
            graph.setXRange(-1,21)
            for i in range(0, len(glist)):
                if cbox.currentText() == 'All' or cbox.currentIndex() == i:   
                    path = pg.arrayToQPath(np.arange(len(glist[i])), glist[i])
                    item = QtGui.QGraphicsPathItem(path)
                    item.setPen(pg.mkPen(color=self.colorlist[i]))
                    graph.addItem(item)




    def make_Graphs(self):
        """Method to initialize the different plots.

        Creates a plot with grid and legend, then adds it to the coresponding layout.
        """
        self.tempplt = pg.plot()
        self.tempplt.setYRange(-1, 100)
        self.tempplt.showGrid(x=True, y=True)
        self.tempplt.addLegend()
        for i in range(0, 10):  
            self.tempplt.plot(np.zeros(10),np.zeros(10), pen=self.colorlist[i], name=self.motornames[i])    
        self.tempplt.addLegend(offset=79)
        #creates a legend by plotting a single point for each motor, just so they show up in the legend.
        #Apparently you are supposed to do it that way...  
        for i in range(10, 20):
            self.tempplt.plot(np.zeros(10),np.zeros(10), pen=self.colorlist[i], name=self.motornames[i])
        self.layout_temp = QtGui.QHBoxLayout()
        self.combobox3= QComboBox()
        self.combobox3.addItem('All')
        for i in self.motornames:
            self.combobox3.addItem(i)
        self._widget.Temperatur.setLayout(self.layout_temp)
        self.layout_temp.addWidget(self.tempplt)
        self.layout_temp.addWidget(self.combobox3)
        self.tempplt.win.hide()

        self.torqueplt = pg.plot()
        self.torqueplt.setYRange(-1, 2)
        self.torqueplt.showGrid(x=True, y=True)
        self.torqueplt.addLegend()
        for i in range(0, 10):
            self.torqueplt.plot(np.zeros(10),np.zeros(10), pen=self.colorlist[i], name=self.motornames[i])
        self.torqueplt.addLegend(offset=79)
        for i in range(10, 20):
            self.torqueplt.plot(np.zeros(10),np.zeros(10), pen=self.colorlist[i], name=self.motornames[i])
        self.layout_torque = QtGui.QHBoxLayout()
        self._widget.Torque.setLayout(self.layout_torque)
        self.layout_torque.addWidget(self.torqueplt)
        self.combobox= QComboBox()
        self.combobox.addItem('All')
        for i in self.motornames:
            self.combobox.addItem(i)
        self.layout_torque.addWidget(self.combobox)
        self.torqueplt.win.hide()

        self.voltageplt = pg.plot()
        self.voltageplt.setYRange(-1, 30)
        self.voltageplt.showGrid(x=True, y=True)
        self.voltageplt.addLegend()
        for i in range(0, 10):
            self.voltageplt.plot(np.zeros(10),np.zeros(10), pen=self.colorlist[i], name=self.motornames[i])
        self.voltageplt.addLegend(offset=79)
        for i in range(10, 20):
            self.voltageplt.plot(np.zeros(10),np.zeros(10), pen=self.colorlist[i], name=self.motornames[i])
        self.layout_voltage = QtGui.QHBoxLayout()
        self.combobox2= QComboBox()
        self.combobox2.addItem('All')
        for i in self.motornames:
            self.combobox2.addItem(i)
        self._widget.Voltage.setLayout(self.layout_voltage)
        self.layout_voltage.addWidget(self.voltageplt)
        self.layout_voltage.addWidget(self.combobox2)
        self.voltageplt.win.hide()
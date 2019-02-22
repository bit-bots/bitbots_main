#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from python_qt_binding.QtWidgets import QMainWindow, QLabel, QTableWidgetItem, QTableWidget,QComboBox, QInputDialog, QLineEdit
import pyqtgraph as pg
from python_qt_binding import loadUi, QtCore, QtGui
from rqt_gui_py.plugin import Plugin

from udp_receiver import Client
from bitbots_hardware_rqt.msg import HardwareMessage

import rospy, rospkg
import numpy as np
import os, math, random
import threading
import json
import getpass, datetime, sys, socket

class HardwareUI(Plugin):
    """Backend for the GUI. Receives UDP data and distributes it across all GUI elements."""
    sig = QtCore.pyqtSignal(str)
    def __init__(self, context):
        """Loads up the master.ui GUI file and adds some additional widgets that cannot be added beforehand. Also sets some necessary variables."""
        super(HardwareUI, self).__init__(context)

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

        self.templist = []
        self.torquelist = []
        self.voltagelist = []

        self._widget = QMainWindow()

        self._robot_ip = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]
        self._robot_port, button_pressed = QInputDialog.getText(self._widget, "Change Port?", \
                "Please select the port you want to listen on. Leave blank for port 5005.", QLineEdit.Normal, "")
        if button_pressed and self._robot_port == "":
            self._robot_port = "5005"
        self.current_tab = 0

        self.rcvthread = ReceiverThread(self._robot_ip, self._robot_port)
        self.rcvthread.start()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('bitbots_hardware_rqt'), 'resource', 'master.ui')
        loadUi(ui_file, self._widget, {})

        self.write_log = False

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

        self._widget.GraphenView.currentChanged.connect(self.current_graph_changed)

        self._widget.checkBox_5.stateChanged.connect(self.write_log_trigger)
        self.make_Graphs()

        self._widget.label_14.setText("IP: "+ [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0])

        context.add_widget(self._widget)
        self._widget.show()

        self.sub = rospy.Subscriber("/hardware_info", HardwareMessage, self.process_data)

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


    def process_data(self, data):
        """Triggered each time we receive an UDP message. Calls methods to update each GUI element"""
        data_json = json.loads(data.data)

        self.set_imu(data_json)
        self.set_topbar(data_json)
        self.set_cpu_tab(data_json)
        self.set_motor_overview(data_json)
        self.set_motor_temperature_graph(data_json)
        self.set_motor_torque_graph(data_json)
        self.set_motor_voltage_graph(data_json)

        #Writes into the logfile, if logging is enabled
        if self.write_log:
            self.logfile.write(data)
            self.logfile.write("\n \n")

    def set_imu(self, data):
        """Updates the IMU/Gyro widgets."""
        self._widget.label_8.setText(str(math.degrees(data['roll'])))
        self._widget.label_9.setText(str(math.degrees(data['pitch'])))
        self._widget.label_10.setText(str(math.degrees(data['yaw'])))

        self.crosshair.move((187+(math.degrees(data['roll'])*400/360)), \
            (187+(math.degrees(data['pitch'])*400/360)))    #To place the crosshair on the circle, circle diametre is 400px, crosshair is 25px wide
        yawx, yawy = self.calcPosInCicle(data['yaw'])
        self.crosshair2.move(yawx, yawy)

    def set_topbar(self, data):
        """Updates the widgets placed in the topbar"""
        self._widget.checkBox.setCheckState(data['button1'])
        self._widget.checkBox.setCheckState(data['button2'])
        timestamp = data['timestamp'].split('.')
        self._widget.label_13.setText(timestamp[0])
        self._widget.RobotState.display(data['state'])

    def set_cpu_tab(self, data):
        """Updates the widgets in the CPU tab"""
        for i in data['cpulist']:
            if i['name'] == 'cpu_name':
                self._widget.tableWidget_3.setItem(0, 0, QTableWidgetItem(i['usage']))      #TODO: Configure this for robots.
                self._widget.tableWidget_3.setItem(1, 0, QTableWidgetItem(i['temp']))
            elif i['name'] == 'cpu_name2':
                self._widget.tableWidget_4.setItem(0, 0, QTableWidgetItem(i['usage']))
                self._widget.tableWidget_4.setItem(1, 0, QTableWidgetItem(i['temp']))

    def set_motor_overview(self, data):
        """Updates the table in the motor overview tab"""
        if self.current_tab == 0:
            self.motorheaderlist = []
            for i in range(0, min(10, len(data['motors']))):
                self.motorheaderlist.append(data['motors'][i]['name'])
                self._widget.tableWidget.setItem(0, i, QTableWidgetItem(data['motors'][i]['message']))
                self._widget.tableWidget.setItem(1, i, QTableWidgetItem(str(round(float(data['motors'][i]['temperature']),2))))
                self._widget.tableWidget.setItem(2, i, QTableWidgetItem(str(round(float(data['motors'][i]['torque']),2))))
                self._widget.tableWidget.setItem(3, i, QTableWidgetItem(str(round(float(data['motors'][i]['input_voltage']),2))))
                self._widget.tableWidget.setItem(4, i, QTableWidgetItem(data['motors'][i]['error_byte']))
                self._widget.tableWidget.setItem(5, i, QTableWidgetItem(str(round(float(data['motors'][i]['position']),2))))


            self._widget.tableWidget.setHorizontalHeaderLabels(self.motorheaderlist)
            self._widget.tableWidget.setEditTriggers(QTableWidget.NoEditTriggers)

            if len(data['motors']) > 10: #Don't try to fill the second table, if for some reason there aren't enough motors
                self.motorheaderlist = []
                for i in range(10, min(20, len(data['motors']))):
                    self.motorheaderlist.append(data['motors'][i]['name'])
                    self._widget.tableWidget_2.setItem(0, i-10, QTableWidgetItem(data['motors'][i]['message']))
                    self._widget.tableWidget_2.setItem(1, i-10, QTableWidgetItem(str(round(float(data['motors'][i]['temperature']),2))))
                    self._widget.tableWidget_2.setItem(2, i-10, QTableWidgetItem(str(round(float(data['motors'][i]['torque']),2))))
                    self._widget.tableWidget_2.setItem(3, i-10, QTableWidgetItem(str(round(float(data['motors'][i]['input_voltage']),2))))
                    self._widget.tableWidget_2.setItem(4, i-10, QTableWidgetItem(data['motors'][i]['error_byte']))
                    self._widget.tableWidget_2.setItem(5, i-10, QTableWidgetItem(str(round(float(data['motors'][i]['position']),2))))

                self._widget.tableWidget_2.setHorizontalHeaderLabels(self.motorheaderlist)
                self._widget.tableWidget_2.setEditTriggers(QTableWidget.NoEditTriggers)

    def set_motor_temperature_graph(self, data):
        """Updates the graph that displays the motors temperature"""
        if self.current_tab == 1:
            self.tempplt.clear()
            for i in range(0, len(data['motors'])):
                if len(self.templist) <= i:
                    self.templist.append([float(data['motors'][i]['temperature'])])
                else: 
                    self.templist[i].append(float(data['motors'][i]['temperature']))
                if self.combobox3.currentText() == 'All' or self.combobox3.currentIndex() == i:   
                    path = pg.arrayToQPath(np.arange(len(self.templist[i])), self.templist[i])
                    item = QtGui.QGraphicsPathItem(path)
                    item.setPen(pg.mkPen(color=self.colorlist[i]))
                    self.tempplt.addItem(item)
                if len(self.templist[i]) > 20:
                    self.templist[i].pop(0)

    def set_motor_torque_graph(self, data):
        """Updates the graph that displays the motors torque"""
        if self.current_tab == 2:
            self.torqueplt.clear()
            for i in range(0, len(data['motors'])):
                if len(self.torquelist) <= i:
                    self.torquelist.append([float(data['motors'][i]['torque'])])
                else: 
                    self.torquelist[i].append(float(data['motors'][i]['torque']))
                if self.combobox.currentText() == 'All' or self.combobox.currentIndex() == i:   
                #if the combobox says 'All', plot all, else only plot the one where the name equals the selected name
                    path = pg.arrayToQPath(np.arange(len(self.torquelist[i])), self.torquelist[i])
                    item = QtGui.QGraphicsPathItem(path)
                    item.setPen(pg.mkPen(color=self.colorlist[i]))
                    self.torqueplt.addItem(item)
                if len(self.torquelist[i]) > 20:
                    self.torquelist[i].pop(0)
            

    def set_motor_voltage_graph(self, data):
        """Updates the graph that displays the motors voltage"""
        if self.current_tab == 3:
            self.voltageplt.clear()
            for i in range(0, len(data['motors'])):
                if len(self.voltagelist) <= i:
                    self.voltagelist.append([float(data['motors'][i]['input_voltage'])])
                else: 
                    self.voltagelist[i].append(float(data['motors'][i]['input_voltage']))
                if self.combobox2.currentText() == 'All' or self.combobox2.currentIndex() == i:   
                    path = pg.arrayToQPath(np.arange(len(self.voltagelist[i])), self.voltagelist[i])
                    item = QtGui.QGraphicsPathItem(path)
                    item.setPen(pg.mkPen(color=self.colorlist[i]))
                    self.voltageplt.addItem(item)
                if len(self.voltagelist[i]) > 20:
                    self.voltagelist[i].pop(0)




    def make_Graphs(self):
        """Method to initialize the different plots.

        Creates a plot with grid and legend, then adds it to the coresponding layout.
        """
        self.tempplt = pg.plot()
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

class ReceiverThread(QtCore.QThread):
    def __init__(self, ip, port, parent=None):
        QtCore.QThread.__init__(self, parent)
        self.ip = ip
        self.port = port

    def run(self):
        self.myclient = Client(self.ip, self.port)
        self.myclient.start()
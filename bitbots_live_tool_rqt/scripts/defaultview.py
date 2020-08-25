import sys
from python_qt_binding.QtCore import Qt, QMetaType, QDataStream, QVariant, pyqtSignal
from python_qt_binding import loadUi
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QTreeWidget, QTreeWidgetItem,QListWidgetItem, \
    QSlider, QGroupBox, QVBoxLayout, QLabel, QLineEdit, QListWidget, QAbstractItemView, QFileDialog, QDoubleSpinBox, QMessageBox, \
    QInputDialog, QShortcut
from python_qt_binding.QtGui import QDoubleValidator, QKeySequence, QPixmap, QTransform

from status_msg import StatusMsg
from quarter_field import QuarterField, RobotInformation


class DefaultView :
    def __init__(self, field, lb_name, lb_nextaction, lb_penalty, \
                 lb_role, lb_statehardware, lb_stateindicatorgreen, lb_stateindicatorred):
        """

        :param field: the given soccerfield
        :param lb_name: label for the name of the robot
        :param lb_nextaction: label for the next action
        :param lb_penalty: label for the penalty duration
        :param lb_role: label for the role of the robot
        :param lb_statehardware: label for the hardware status
        :param lb_stateindicatorgreen: label for a colored indicator (in penalty or not)
        :param lb_stateindicatorred: label for a colored indicator (in penalty or not)
        """
        # decodes the integer for the corresponding role
        self.roleDecoder = {0: 'IDLING', 1: 'OTHER', 2: 'STRIKER', 3: 'SUPPORTER', 4: 'DEFENDER', 5: 'GOALIE'}
        # decodes the integer for the corresponding action
        self.actionDecoder = {0: 'Undefinded', 1: 'Positioning', 2: 'Going to ball', 3: 'Trying to score', 4: 'Waiting'}
        # decodes the integer for the corresponding hardware state
        self.stateDecoder = {0: 'CONTROLLABLE', 1: 'FALLING', 2: 'FALLEN', 3: 'GETTING_UP', 4: 'ANIMATION_RUNNING', 5: 'STARTUP', \
                             6: 'SHUTDOWN', 7: 'PENALTY', 8: 'PENALTY_ANIMATION', 9: 'RECORD', 10: 'WALKING', 11: 'MOTOR_OFF', \
                             12: 'HCM_OFF', 13: 'HARDWARE_PROBLEM', 14: 'PICKED_UP'}

        # elements for stats of rob
        self.lbName = lb_name
        self.lbNextAction = lb_nextaction
        self.lbPenalty = lb_penalty
        self.lbRole = lb_role
        self.lbStateHardware = lb_statehardware
        self.lbStateIndicatorGreen = lb_stateindicatorgreen
        self.lbStateIndicatorRed = lb_stateindicatorred
        self.field = field

        # hides the indicators by default
        self.lbStateIndicatorGreen.hide()
        self.lbStateIndicatorRed.hide()


    # Label updates
    def actualizePenaltylabel(self, data):
        """
        updates the penalty label
        :param data: a dictionary with the transmitted information
        :return:
        """
        if data.has_key(StatusMsg.label_penalty_rest):
            self.lbPenalty.setText(str(data.get(StatusMsg.label_penalty_rest)))

    def acutalizeRolelabel(self, data):
        """
        updates the role label
        :param data: a dictionary with the transmitted information
        :return:
        """
        if data.has_key(StatusMsg.label_role):
            self.lbRole.setText(self.roleDecoder.get(data.get(StatusMsg.label_role)))

    def acutalizeActionlabel(self, data):
        """
        updates the action label
        :param data: a dictionary with the transmitted information
        :return:
        """
        if data.has_key(StatusMsg.label_action):
            self.lbNextAction.setText(self.actionDecoder.get(data.get(StatusMsg.label_action)))


    def actualizeHardwarelabel (self, data):
        """
        updates the hardware label
        :param data: a dictionary with the transmitted information
        :return:
        """
        if data.has_key(StatusMsg.label_state):
            self.lbStateHardware.setText(self.stateDecoder.get(data.get(StatusMsg.label_state)))


    def actualizeActivitystate(self, data):
        """
        updates the indicator green or red depending on the penalty state the robot is in
        :param data:a dictionary with the transmitted information
        :return:
        """
        if data.has_key(StatusMsg.label_penalized):
            self.lbStateIndicatorRed.raise_()
            if data.get(StatusMsg.label_penalized) == True:
                self.lbStateIndicatorGreen.hide()
                self.lbStateIndicatorRed.show()
            else:
                self.lbStateIndicatorGreen.show()
                self.lbStateIndicatorRed.hide()

    def setName (self, robotID):
        """
        sets the name of the robot
        :param robotID: a dictionary with the transmitted information
        :return:
        """
        self.lbName.setText(robotID)

    def setStatusMsg(self, robotID, data):
        """
        sets all the labels
        :param robotID: the name of the robot
        :param data: a dictionary with the transmitted information
        :return:
        """
        self.actualizePenaltylabel(data)
        self.acutalizeRolelabel(data)
        self.acutalizeActionlabel(data)
        self.actualizeActivitystate(data)
        self.setName(robotID)
        self.actualizeHardwarelabel(data)





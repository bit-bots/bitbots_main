import sys
#from PyQt4 import QtGui, QtCore

from status_msg import StatusMsg
from single_field import SingleField


class InformationTab :
    def __init__(self, tab_no, field, rolelabel, penaltylabel, hardwarelabel, actionlabel, hideallcheck, targetcheck, \
                 ballcheck, teammatecheck, opponentcheck, undefcheck, activitystateGreen, activitystateRed):
        """

        :param tab_no: the number of the robot tab
        :param field: the soccer field
        :param rolelabel: label for the robot role
        :param penaltylabel: label for the penalty
        :param hardwarelabel: label for the hardware status
        :param actionlabel: label for the next action
        :param hideallcheck: check box to hide everything
        :param targetcheck: check box to hide the target
        :param ballcheck: check box to hide the ball
        :param teammatecheck: check box to hide teammates
        :param opponentcheck: check box to hide opponents
        :param undefcheck: check box to hide undefined obstacles
        :param activitystateGreen: green indicator
        :param activitystateRed: red indicator
        """
        #tab index [0;3]
        self.index = tab_no

        # decodes the integer for the corresponding role
        self.roleDecoder = {0: 'IDLING', 1: 'OTHER', 2: 'STRIKER', 3: 'SUPPORTER', 4: 'DEFENDER', 5: 'GOALIE'}
        # decodes the integer for the corresponding action
        self.actionDecoder = {0: 'Undefinded', 1: 'Positioning', 2: 'Going to ball', 3: 'Trying to score', 4: 'Waiting'}

        self.stateDecoder = {0: 'CONTROLLABLE', 1: 'FALLING', 2: 'FALLEN', 3: 'GETTING_UP', 4: 'ANIMATION_RUNNING', 5: 'STARTUP', \
                             6: 'SHUTDOWN', 7: 'PENALTY', 8: 'PENALTY_ANIMATION', 9: 'RECORD', 10: 'WALKING', 11: 'MOTOR_OFF', \
                             12: 'HCM_OFF', 13: 'HARDWARE_PROBLEM', 14: 'PICKED_UP'}

        # Labels, get msg
        self.rolelabel = rolelabel
        self.penaltylabel = penaltylabel
        self.hardwarelabel = hardwarelabel
        self.actionlabel = actionlabel

        # Checkboxes, default checked, but hide all unchecked
        self.hideallcheck = hideallcheck
        self.hideallcheck.setChecked(False) # hide all undechecked
        self.hideallcheck.stateChanged.connect(lambda: self.hideallstate())

        self.targetcheck = targetcheck
        self.targetcheck.setChecked(True)
        self.targetcheck.stateChanged.connect(lambda: self.targetstate())

        self.ballcheck = ballcheck
        self.ballcheck.setChecked(True)
        self.ballcheck.stateChanged.connect(lambda: self.ballstate())

        self.teammatecheck = teammatecheck
        self.teammatecheck.setChecked(True)
        self.teammatecheck.stateChanged.connect(lambda: self.teammatestate())

        self.opponentcheck = opponentcheck
        self.opponentcheck.setChecked(True)
        self.opponentcheck.stateChanged.connect(lambda: self.opponentstate())

        self.undefcheck = undefcheck
        self.undefcheck.setChecked(True)
        self.undefcheck.stateChanged.connect(lambda: self.undefinedstate())

        self.activitystateGreen = activitystateGreen
        self.activitystateGreen.hide()
        self.activitystateRed = activitystateRed
        self.activitystateRed.show()

        self.field = field




    # Labels
    def actualizePenaltylabel(self, data):
        """
        updates the penalty label
        :param data: a dictionary with the transmitted information
        :return:
        """
        if data.has_key(StatusMsg.label_penalty_rest):
            self.penaltylabel.setText(str(data.get(StatusMsg.label_penalty_rest)))


    def acutalizeRolelabel (self, data):
        """
        updates the role label
        :param data: a dictionary with the transmitted information
        :return:
        """
        if data.has_key(StatusMsg.label_role):
            self.rolelabel.setText(self.roleDecoder.get(data.get(StatusMsg.label_role)))

    def acutalizeActionlabel (self, data):
        """
        updates the action label
        :param data: a dictionary with the transmitted information
        :return:
        """
        if data.has_key(StatusMsg.label_action):
            self.actionlabel.setText(self.actionDecoder.get(data.get(StatusMsg.label_action)))


    def actualizeHardwarelabel (self, data):
        """
        updates the hardware label
        :param data: a dictionary with the transmitted information
        :return:
        """
        if data.has_key(StatusMsg.label_state):
            self.hardwarelabel.setText(self.stateDecoder.get(data.get(StatusMsg.label_state)))



    def actualizeActivitystate(self, data):
        """

        :param data: a dictionary with the transmitted information
        :return:
        """
        if data.has_key(StatusMsg.label_penalized):
            self.activitystateRed.raise_()
            if data.get(StatusMsg.label_penalized) == True:
                self.activitystateGreen.hide()
                self.activitystateRed.show()
            else:
                self.activitystateGreen.show()
                self.activitystateRed.hide()


    def setStatusMsg(self, data):
        """

        :param data: a dictionary with the transmitted information
        :return:
        """
        self.actualizePenaltylabel(data)
        self.acutalizeRolelabel(data)
        self.acutalizeActionlabel(data)
        self.actualizeActivitystate(data)
        self.actualizeHardwarelabel(data)


    # Filters for objects on field
    def hideallstate(self):
        """
        hide every label on the field
        :return:
        """
        if self.hideallcheck.isChecked() == True:
            self.field.setOwnRobotsVisibility(False, self.index)
            self.field.setPathVisibility(False, self.index)
            self.field.setBallVisibility(False, self.index)
            self.field.setTeammateVisibility(False, self.index)
            #self.field.setPathVisibility(False, self.index)
            self.field.setOpponentVisibility(False, self.index)
            self.field.setUndefVisibility(False, self.index)
            self.ballcheck.setChecked(False)
            self.teammatecheck.setChecked(False)
            self.opponentcheck.setChecked(False)
            self.undefcheck.setChecked(False)
            self.targetcheck.setChecked(False)
        else:
            self.field.setOwnRobotsVisibility(True, self.index)
            self.field.setPathVisibility(True, self.index)
            self.field.setBallVisibility(True, self.index)
            self.field.setTeammateVisibility(True, self.index)
            #self.field.setPathVisibility(True, self.index)
            self.field.setOpponentVisibility(True, self.index)
            self.field.setUndefVisibility(True, self.index)
            self.ballcheck.setChecked(True)
            self.teammatecheck.setChecked(True)
            self.opponentcheck.setChecked(True)
            self.undefcheck.setChecked(True)
            self.targetcheck.setChecked(True)

    def targetstate(self):
        if self.targetcheck.isChecked() == True:
            self.field.setPathVisibility(True, self.index)
        else:
            self.field.setPathVisibility(False, self.index)

    def ballstate(self):
        if self.ballcheck.isChecked() == True:
            self.field.setBallVisibility(True, self.index)
        else:
            self.field.setBallVisibility(False, self.index)

    def teammatestate(self):
        if self.teammatecheck.isChecked() == True:
            self.field.setTeammateVisibility(True, self.index)
        else:
            self.field.setTeammateVisibility(False, self.index)

    def opponentstate(self):
        if self.opponentcheck.isChecked() == True:
            self.field.setOpponentVisibility(True, self.index)
        else:
            self.field.setOpponentVisibility(False, self.index)

    def undefinedstate(self):
        if self.undefcheck.isChecked() == True:
            self.field.setUndefVisibility(True, self.index)
        else:
            self.field.setUndefVisibility(False, self.index)




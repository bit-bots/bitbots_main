import sys
#import msvcrt

from python_qt_binding.QtCore import Qt, QMetaType, QDataStream, QVariant, pyqtSignal
from python_qt_binding import loadUi
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QTreeWidget, QTreeWidgetItem,QListWidgetItem, \
    QSlider, QGroupBox, QVBoxLayout, QLabel, QLineEdit, QListWidget, QAbstractItemView, QFileDialog, QDoubleSpinBox, QMessageBox, \
    QInputDialog, QShortcut
#from python_qt_binding.QtGui import QDoubleValidator, QKeySequence, QPixmap, QTransform, QKeySequence#, QShortcut
from python_qt_binding.QtGui import *

from status_msg import StatusMsg
from quarter_field import QuarterField

class MainWindow :
    def __init__(self, field, fields_4,  sideToggle, robStatusTab, lb_uebersicht, lb_gamestate, lb_gamescore, lb_extratime):

        self.lbHalf = lb_gamestate
        self.lbExtraTime = lb_extratime
        self.lbGameScore = lb_gamescore
        self.tabManager = lb_uebersicht
        self.robStatusTab = robStatusTab
        self.fields_4 = fields_4
        self.field = field
        self.sideToggle = sideToggle

        #self.lbHalf.setText("GamePhase")
        
        #Key-Shortcut for switching tabs, lbGamePhase just because we need to connect it to one element
        QShortcut(QKeySequence("Tab"), self.lbHalf, self.shortcut)
        QShortcut(QKeySequence("1"), self.lbHalf, lambda : self.setTab1(0))
        QShortcut(QKeySequence("2"), self.lbHalf, lambda : self.setTab1(1))
        QShortcut(QKeySequence("3"), self.lbHalf, lambda : self.setTab1(2))
        QShortcut(QKeySequence("4"), self.lbHalf, lambda : self.setTab1(3))

        sideToggle.stateChanged.connect(lambda: self.changeSides())

    def changeSides(self):
        self.field.setSide(not self.sideToggle.isChecked())
        for f in self.fields_4:
            f.setSide(not self.sideToggle.isChecked())

    def shortcut(self):
        self.tabManager.setCurrentIndex(1 - self.tabManager.currentIndex())

    def setTab1(self, nr):
        self.robStatusTab.setCurrentIndex(nr)

                            
    #methods for updating labels
    def updateHalbzeit (self, data):
        firstHalf = data.get(StatusMsg.label_first_half)
        if firstHalf:
            self.lbHalf.setText("1. halftime")
        else:
            self.lbHalf.setText("2. halftime")


    def updateRemainingTime (self, data):
        secs_total = data.get(StatusMsg.label_remain_secs)
        mins = int(secs_total / 60)
        secs = secs_total % 60
        self.lbExtraTime.setText('{:02}'.format(mins) + ":" + '{:02}'.format(secs))
        
    #combines both own and rival score to e.g. 5:5
    def updateGameScore (self, data):
        self.lbGameScore.setText(str(data.get(StatusMsg.label_own_score)) +" : "+ str(data.get(StatusMsg.label_rival_score)))


    def setStatusMsg(self, data):
        if data.has_key(StatusMsg.label_first_half):
            self.updateHalbzeit(data)
        if data.has_key(StatusMsg.label_remain_secs):
            self.updateRemainingTime(data)
        if data.has_key(StatusMsg.label_own_score):
            self.updateGameScore(data)

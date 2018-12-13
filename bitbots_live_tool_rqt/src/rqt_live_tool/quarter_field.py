import numpy as np
import math
from python_qt_binding.QtCore import Qt, QMetaType, QDataStream, QVariant, pyqtSignal
from python_qt_binding import loadUi
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QTreeWidget, QTreeWidgetItem, QListWidgetItem, \
    QSlider, QGroupBox, QVBoxLayout, QLabel, QLineEdit, QListWidget, QAbstractItemView, QFileDialog, QDoubleSpinBox, \
    QMessageBox, \
    QInputDialog, QShortcut
from python_qt_binding.QtGui import QDoubleValidator, QKeySequence, QPixmap, QTransform

import os
import rospkg, rospy
import time
import multiprocessing as mp
import yaml

from label_pool import BallPool, RobotPool, CrossPool, UndefinedPool, TeammatePool, AngularLabel, OpponentPool, Arrowlabel

from status_msg import StatusMsg
from trajectory_msg import TrajectoryMsg
from detection_msg import DetectionMsg
from position_msg import PositionMsg

from name import Name



# class for information of one robot on "his" field
class RobotInformation:

    def __init__(self, robotID, frame, colorString):  # robot-ID, frame (1-4), colour of that rob
        self.id = robotID
        self.frame = frame
        self.color = colorString

        self.rob_label = None  # Qlabels
        self.ang_label = None
        self.arrow_label = None

        self.pixmap = None

        self.team_color = ""
        self.oppo_color = ""

        self.x = 0
        self.y = 0
        self.angel = 0

        # positions of ball, obstacles, teammates, opponnents, maybe path
        self.ballPositions = []  # list of triple: x, y, confidence
        self.currentBallLabel = []
        self.obstacles = []  # list of triple: x, y, confidence
        self.currentObstacleLabel = []
        self.teammates = []  # list of triple: x, y, confidence
        self.currentTeammateLabel = []
        self.opponents = []  # list of triple: x, y, confidence
        self.currentOpponentsLabel = []
        self.path = []  # list of tuple: x, y
        self.currentPathLabel = []



class QuarterField:
    # fieldframe = frame, which contains game-field and all information of that roboter
    def __init__(self, gameframe, fieldImageLabel):
        # receive parameters
        self.frame = gameframe
        self.field = fieldImageLabel

        # set yaml config file
        rp = rospkg.RosPack()
        ip_filename = os.path.join(rp.get_path('rqt_live_tool'), 'resource', 'ip_config.yaml')
        with open(ip_filename, "r") as file:
            config = yaml.load(file)
            self.fieldfilename = config.get("FIELD_IMAGE")
            self.field_scale_global = config.get("FIELD_SCALE")
            self.default_positions = config.get("DEFAULT_POSITIONS")
        file.close()

        # debug
        #print("Config: " + self.fieldfilename + ", " + str(self.field_scale_global))

# set & scale field ====================================================
        rp = rospkg.RosPack()
        self.fieldfilename = os.path.join(BallPool.rp.get_path('rqt_live_tool'), 'resource', self.fieldfilename)
        self.fieldPixmap = QPixmap(self.fieldfilename)
        self.field.setPixmap(self.fieldPixmap)

        field_width = self.fieldPixmap.width()
        field_height = self.fieldPixmap.height()

        self.field_aspect = float(field_width) / float(field_height)
        self.frame_aspect = float(self.frame.width()) / float(self.frame.height())

        self.field_border = 10  # margin to frame

        self.transform = QTransform()

        self.icon_timeout = 3

        self.fieldIsSwitched = False # for function "switch sides"

        if self.field_aspect >= self.frame_aspect:
            self.field_size_x = self.frame.width() - self.field_border * 2
            self.field_size_y = self.field_size_x / self.field_aspect
        else:
            self.field_size_y = self.frame.height() - self.field_border * 2
            self.field_size_x = self.field_size_y / self.field_aspect

        self.field.setScaledContents(True)
        self.field.setFixedSize(self.field_size_x, self.field_size_y)

        self.field_scale = float(self.field_size_x) / float(field_width)
        #print(self.field_scale)

        self.screenMidX = int(self.frame.width() / 2)
        self.screenMidY = int(self.frame.height() / 2)

        self.field.move(self.screenMidX - int(self.field.width() / 2), self.screenMidY - int(self.field.height() / 2))
 
 # =================================================================================

        self.colors = ["red", "green", "yellow", "blue"]
        self.team_colors = {3: "cyan", 2: "magenta"}

        self.op_color = None
        self.mate_color = None
        self.own_color = None
        self.myRobo_color = None

        self.robots = {} # dict where keys are robot IDs
        self.ball_pool = BallPool(self.frame, size=32)
        self.rob_pool = RobotPool(self.frame, size=36)
        self.opp_pool = OpponentPool(self.frame, size=32)
        self.team_pool = TeammatePool(self.frame, size=32)
        self.cross_pool = CrossPool(self.frame, size=17)
        self.undef_pool = UndefinedPool(self.frame, size=32)
        #self.cyanmagenta_pool = CyanMagentaPool(self.frame, size=26)
        #rob = self.rob_pool.getRobotLabel("green")  # farbe nur zum testen, sonst uebergabe einer Farbe
        #self.rob_pool.returnRobotLabel(rob, ("green"))

    def setSide(self, default):
        self.fieldIsSwitched = not default

    # helping method for convert meters to UI-values
    def _meter_to_UI(self, val):
        return (val / self.field_scale_global) * self.field_scale

    # add the roboter
    def addRobot(self, robotID, color=""):
        if color == "":
            color = self.colors.pop()
        rob_info = RobotInformation(robotID, self.frame, color)
        rob_info.rob_label = self.rob_pool.getRobotLabel(color)
        rob_info.pixmap = rob_info.rob_label.pixmap()  # saving original pixmap for rotation
        rob_info.rob_label.update()
        rob_info.ang_label = AngularLabel(self.frame)
        rob_info.arrow_label = Arrowlabel(self.frame)
        
        self.frame.update()
        self.robots[robotID] = rob_info

        defpos = {"x": 0, "y": 0}
        if len(self.default_positions) > 0:
            defpos = self.default_positions.pop()
        self.setRobotPosition(robotID, defpos.get("x"), defpos.get("y"), 0)  # sets default position

    # sets the robots position in meter!!!!
    def setRobotPosition(self, robotID, x, y, angle):
        if self.robots.has_key(robotID):
            rob = self.robots.get(robotID)
            if x != None:
                if self.fieldIsSwitched:
                    rob.x = -x
                else:
                    rob.x = x
            if y != None:
                if self.fieldIsSwitched:
                    rob.y = -y
                else:
                    rob.y = y
            if angle != None:
                if self.fieldIsSwitched:
                    rob.angel = -angle + self.degToRadians(180)
                else:
                    rob.angel = -angle

            transform = QTransform()
            transform.rotateRadians(rob.angel)

            rotPixMap = QPixmap(rob.rob_label.originalPixmap).transformed( transform )
            rob.rob_label.setPixmap(rotPixMap)
            rob.rob_label.setScaledContents(True)

            addScale = (abs(math.sin(rob.angel*2))) * (math.sqrt(2) - 1)
            newsize = self.rob_pool.size + (self.rob_pool.size*addScale)

            rob.rob_label.setFixedSize(newsize, newsize)
            
            rob.rob_label.setScaledContents(True)
            rob.rob_label.move(self.screenMidX - int(rob.rob_label.width() / 2) + self._meter_to_UI(rob.x), \
                               self.screenMidY - int(rob.rob_label.height() / 2) - self._meter_to_UI(rob.y))
            rob.ang_label.move( rob.rob_label.x() + int(rob.rob_label.width() / 2) - rob.ang_label.width() / 2,\
                                rob.rob_label.y() + int(rob.rob_label.height() / 2) - rob.ang_label.height() / 2)

            rob.ang_label.setAngles(self.radToDeg(-rob.angel), None)

            rob.arrow_label.move(rob.rob_label.x() + int(rob.rob_label.width() / 2) - rob.arrow_label.width() / 2, \
                                 rob.rob_label.y() + int(rob.rob_label.height() / 2) - rob.arrow_label.height() / 2)

            rob.arrow_label.setRoboAngle(self.radToDeg(rob.angel))

            #rob.rob_label.update()
            #rob.rob_label.repaint()

    def setPathsFor(self, robotID, listPaths):
        if self.robots.has_key(robotID):
            rob = self.robots.get(robotID)
            for i in range(len(rob.currentPathLabel)):
                self.cross_pool.returnCrossLabel( rob.currentPathLabel.pop(), rob.color )
            for pos in listPaths:
                cross = self.cross_pool.getCrossLabel( rob.color )
                cross.move(self.screenMidX - int(cross.width() / 2) + self._meter_to_UI(pos[0]), \
                          self.screenMidY - int(cross.height() / 2) + self._meter_to_UI(pos[1]))
                cross.show()
                rob.currentPathLabel.append(cross)

    # receive and set message data, cyan & magenta for mates
    def setTeammatePosition(self, robotID, listMatePositions, lastUpdate):
        if self.robots.has_key(robotID):
            rob = self.robots.get(robotID)
            time_secs = rospy.get_rostime().secs

            for i in range(len(rob.currentTeammateLabel)):
                self.team_pool.returnTeammateLabel(rob.currentTeammateLabel.pop(), rob.color)

            for pos in listMatePositions:
                if time_secs - lastUpdate < self.icon_timeout:
                    tpx, tpy = self.relToAbs(rob.x, rob.y, rob.angel, pos[0], pos[1])
                    #mate = self.team_pool.getTeammateLabel(pos[2])# Index 2 = always cyan OR magenta
                    mate = self.team_pool.getTeammateLabel(self.team_colors.get(rob.team_color))# Index 2 = always cyan OR magenta

                    mate.move(self.screenMidX - int(mate.width() / 2) + self._meter_to_UI(tpx), \
                              self.screenMidY - int(mate.height() / 2) + self._meter_to_UI(tpy))
                    mate.show()
                    rob.currentTeammateLabel.append(mate)

    def setOpponentPosition(self, robotID, listOpponentPositions, lastUpdate):
        if self.robots.has_key(robotID):
            rob = self.robots.get(robotID)
            time_secs = rospy.get_rostime().secs

            for i in range(len(rob.currentOpponentsLabel)):
                self.opp_pool.returnOpponentLabel(rob.currentOpponentsLabel.pop(), rob.color)

            for pos in listOpponentPositions:
                if time_secs - lastUpdate < self.icon_timeout:
                    oppx, oppy = self.relToAbs(rob.x, rob.y, rob.angel, pos[0], pos[1])
                    #op = self.opp_pool.getOpponentLabel(pos[2])
                    op = self.opp_pool.getOpponentLabel(self.team_colors.get(rob.oppo_color))
                    op.move(self.screenMidX - int(op.width() / 2) + self._meter_to_UI(oppx), \
                              self.screenMidY - int(op.height() / 2) + self._meter_to_UI(oppy))
                    op.show()
                    rob.currentOpponentsLabel.append(op)

    # obstacles are undefined
    def setObstaclePosition(self, robotID, listObstaclePositions, lastUpdate):
        if self.robots.has_key(robotID):
            rob = self.robots.get(robotID)
            time_secs = rospy.get_rostime().secs
            for i in range(len(rob.currentObstacleLabel)):
                self.undef_pool.returnUndefLabel(rob.currentObstacleLabel.pop(), rob.color)

            for pos in listObstaclePositions:
                if time_secs - lastUpdate < self.icon_timeout:
                    opx, opy = self.relToAbs(rob.x, rob.y, rob.angel, pos[0], pos[1])
                    obstacle = self.undef_pool.getUndefLabel("grey")
                    obstacle.move(self.screenMidX - int(obstacle.width() / 2) + self._meter_to_UI(opx), \
                              self.screenMidY - int(obstacle.height() / 2) + self._meter_to_UI(opy))
                    obstacle.show()
                    rob.currentObstacleLabel.append(obstacle)


    def setBallsFor(self, robotID, listPositions, lastUpdate):
        if self.robots.has_key(robotID):
            rob = self.robots.get(robotID)
            time_secs = rospy.get_rostime().secs

            for i in range(len(rob.currentBallLabel)):
                self.ball_pool.returnBallLabel( rob.currentBallLabel.pop(), rob.color )
            for pos in listPositions:
                if time_secs - lastUpdate < self.icon_timeout:
                    bpx, bpy = self.relToAbs(rob.x, rob.y, rob.angel, pos[0], pos[1])
                    ball = self.ball_pool.getBallLabel( rob.color )
                    ball.move(self.screenMidX - int(ball.width() / 2) + self._meter_to_UI(bpx), \
                              self.screenMidY - int(ball.height() / 2) + self._meter_to_UI(bpy))
                    rob.currentBallLabel.append(ball)


    # Message decoding ============================================================================
    def setTrajectoryMsg(self, robotID, data):
        if not self.robots.has_key(robotID):
            self.addRobot(robotID)
        # 5 new lines 6.9.
        if data.has_key("x"):
            self.setPathsFor(robotID, [(data.get("x"), data.get("y"), data.get("angle"))])

        # Twist vectors: rotation
        if data.has_key(TrajectoryMsg.label_rotateVel):
            rob = self.robots.get(robotID)
            rob.ang_label.setAngles(self.radToDeg(-rob.angel),
                                    self.radToDeg(data.get(TrajectoryMsg.label_rotateVel)))

        # Twist vectors: linear velocity
        if data.has_key(TrajectoryMsg.label_moveVelX):
            rob = self.robots.get(robotID)
            rob.arrow_label.setLinearAngle(self._meter_to_UI(data.get(TrajectoryMsg.label_moveVelX)),
                                           self._meter_to_UI(data.get(TrajectoryMsg.label_moveVelY)))
            rob.arrow_label.move(rob.rob_label.x() + int(rob.rob_label.width() / 2) - rob.arrow_label.width() / 2, \
                                 rob.rob_label.y() + int(rob.rob_label.height() / 2) - rob.arrow_label.height() / 2)
            rob.arrow_label.setRoboAngle(self.radToDeg(rob.angel))
            
    def setDetectionMsg(self, robotID, data):
        if not self.robots.has_key(robotID):
            self.addRobot(robotID)
        rob = self.robots.get(robotID)
        if data.has_key(DetectionMsg.label_ball_info):
            self.setBallsFor(robotID, [(data.get(DetectionMsg.label_ball_info).get("x"), data.get(DetectionMsg.label_ball_info).get("y"))], \
                             data.get(DetectionMsg.label_ball_info).get(Name.last_update))

        #gamestate translate to obstacle color
        #if self.own_color == 0:
        #    self.myRobo_color = 3
        #elif self.own_color == 1:
        #    self.myRobo_color = 2

        if data.has_key(DetectionMsg.label_obstacles):
            lsOpponents = []
            lsUndefined = []
            lsMates = []

            for ob in data.get(DetectionMsg.label_obstacles):
                ob_color = ob.get(DetectionMsg.label_obstacle_info).get(DetectionMsg.label_color)
                if ob_color == rob.team_color: #mitspieler +2 weil die farben
                    lsMates.append([ob.get(DetectionMsg.label_obstacle_pos).get("x"), ob.get(DetectionMsg.label_obstacle_pos).get("y")])
                elif ob_color == rob.oppo_color: #gegner
                    lsOpponents.append([ob.get(DetectionMsg.label_obstacle_pos).get("x"), ob.get(DetectionMsg.label_obstacle_pos).get("y")])
                else: #undefined
                    lsUndefined.append([ob.get(DetectionMsg.label_obstacle_pos).get("x"), ob.get(DetectionMsg.label_obstacle_pos).get("y")])


            """
            for ob in data.get(DetectionMsg.label_obstacles):
                if self.own_color != None:

                    if self.myRobo_color == ob.get("info").get("color"):
                        lsMates.append([ob.get("position").get("x"), ob.get("position").get("y"), self.mate_color])
                    elif self.myRobo_color != ob.get("info").get("color"):

                        lsOpponents.append([ob.get("position").get("x"), ob.get("position").get("y"), self.op_color])
                    else :

                        lsUndefined.append([ob.get("position").get("x"), ob.get("position").get("y")])
                else :

                    lsUndefined.append([ob.get("position").get("x"), ob.get("position").get("y")])
            """

            self.setTeammatePosition(robotID, lsMates, data.get(DetectionMsg.label_last_obstacle_update))
            self.setObstaclePosition(robotID, lsUndefined, data.get(DetectionMsg.label_last_obstacle_update))
            self.setOpponentPosition(robotID, lsOpponents, data.get(DetectionMsg.label_last_obstacle_update))


    def setPositionMsg(self, robotID, data):
        if not self.robots.has_key(robotID):
            self.addRobot(robotID)

        if (data.has_key(PositionMsg.label_pos)):
             self.setRobotPosition(robotID, data.get(PositionMsg.label_pos).get("x"), data.get(PositionMsg.label_pos).get("y"),\
                                   data.get(PositionMsg.label_orientation).get(PositionMsg.label_yaw))


    def setStatusMsg(self, robotID, data):
        if not self.robots.has_key(robotID):
            self.addRobot(robotID)
            print("add robot: " + robotID)
        if data.has_key(StatusMsg.labelTeamColor):
            rob = self.robots.get(robotID)
            statusColor = data.get(StatusMsg.labelTeamColor)
            statusColor = (1 - statusColor) + 2 # /gamestate und /obstacles have different constants for colors
            oppoColor = data.get(StatusMsg.labelTeamColor) + 2
            rob.team_color = statusColor
            rob.oppo_color = oppoColor
            #print(rob.team_color, rob.oppo_color)


    """
    def setStatusMsg(self, robotID, data):
        if not self.robots.has_key(robotID):
            self.addRobot(robotID)
        # setting cyan & magenta colors for mates/opponents
        self.own_color = data.get("team_color")
        if self.own_color != None:
            if self.own_color == 0:
                self.mate_color = "cyan"
                self.op_color = "magenta"
            elif self.own_color == 1:
                self.op_color = "cyan"
                self.mate_color = "magenta"
    """
                
                
# Helper ===================================================================

    def vec_rotate(self, x, y, angle_rad):
        xneu = x * math.cos(angle_rad) - y * math.sin(angle_rad)
        yneu = y * math.cos(angle_rad) + x * math.sin(angle_rad)
        return [xneu, yneu]

    def relToAbs(self, fromx, fromy, fromAng, relx, rely):
        rx, ry = self.vec_rotate(relx, rely, fromAng )#- self.degToRadians(90))
        return (fromx + rx, fromy + ry)

    def degToRadians(self, deg):
        return deg / 57.29578

    def radToDeg(self, rads):
        return rads * 57.29578
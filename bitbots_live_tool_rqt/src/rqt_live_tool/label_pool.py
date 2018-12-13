import rospkg
from python_qt_binding.QtCore import Qt, QMetaType, QDataStream, QVariant, pyqtSignal, QLineF, QPointF
from python_qt_binding import loadUi
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QTreeWidget, QTreeWidgetItem,QListWidgetItem, \
    QSlider, QGroupBox, QVBoxLayout, QLabel, QLineEdit, QListWidget, QAbstractItemView, QFileDialog, QDoubleSpinBox, QMessageBox, \
    QInputDialog, QShortcut
from python_qt_binding.QtGui import QDoubleValidator, QKeySequence, QPixmap, QPainter, QPen, QColor, QPolygonF, QTransform, QBrush
import os
import math




class BallPool:

    rp = rospkg.RosPack()

    def _getBallFile(self, colorStr):
        """
        gets the path for the ball file
        :param colorStr: color of the ball as string
        :return:
        """
        #return os.path.join(BallPool.rp.get_path('rqt_live_tool'), 'resource', 'ui_images', "ball-" + colorStr + "_big66x66.png")
        return os.path.join(BallPool.rp.get_path('rqt_live_tool'), 'resource', 'ui_images', "new_icons", "ball_" + colorStr + ".png")

    def __init__(self, frame, size=64):
        """

        :param frame: the frame of the ball
        :param size: size of the ball
        """
        self.pool = {"blue": [], "green": [], "red": [], "yellow": [], "cyan": [], "magenta": []}
        self.frame = frame
        self.size = size

    def getBallLabel(self, colorStr):
        """
        sets the ball label
        :param colorStr: the color fo the ball as string
        :return:
        """
        ls = self.pool.get(colorStr)
        if ls == []:
            pxmap = QPixmap(self._getBallFile(colorStr))
            ball = QLabel(parent=self.frame)
            ball.setPixmap(pxmap)
            ball.setScaledContents(True)
            ball.setFixedSize(self.size, self.size)
            ball.show()
            return ball
        else:
            ball = ls.pop()
            ball.show()
            return ball

    def returnBallLabel(self, ball, colorStr):
        """
        returns the finished ball label
        :param ball: the ball
        :param colorStr: color as string
        :return:
        """
        self.pool[colorStr].append(ball)
        ball.hide()

class RobotWidget(QWidget):

    def __init__(self, parent):
        """

        :param parent: parent
        """
        super(RobotWidget, self).__init__(parent)
        self.angle = 0
        self._pixmap = None

    def setPixmap(self, pixmap):
        """
        sets the pixmap
        :param pixmap: the pixmap
        :return:
        """
        self._pixmap = pixmap

    def setScaledContents(self, bool):
        None

    def pixmap(self):
        """
        returns pixmap
        :return:
        """
        return self._pixmap

    def paintEvent(self, event):
        painter = QPainter(self)
        #painter.setPen(QtCore.Qt.black)
        #painter.translate(self.x(), self.y())
        painter.rotate(self.angle)
        painter.drawPixmap(0, 0, self.width(), self.height(), self._pixmap)
        painter.end()

class RobotPool:
    rp = rospkg.RosPack()

    def _getRobFile(self, colorStr):
        """
        sets the path for the robot file
        :param colorStr: color as string
        :return:
        """
        #return os.path.join(BallPool.rp.get_path('rqt_live_tool'), 'resource', 'ui_images', "Rob_" + colorStr + "-big.png")
        return os.path.join(BallPool.rp.get_path('rqt_live_tool'), 'resource', 'ui_images', "new_icons", "player_" + colorStr + ".png")

    def __init__(self, frame, size=46):
        """

        :param frame: frame of robot
        :param size: size of robot
        """
        self.pool = {"blue": [], "green": [], "red": [], "yellow": []}
        self.frame = frame
        self.size = size

    def getRobotLabel(self, colorStr):
        """
        initiates the robot label
        :param colorStr: color as string
        :return:
        """
        ls = self.pool.get(colorStr)
        if ls == []:
            pxmap = QPixmap(self._getRobFile(colorStr))
            rob = QLabel(parent=self.frame)
            rob.setPixmap(pxmap)
            rob.originalPixmap = QPixmap(pxmap)
            rob.setScaledContents(True)
            rob.setFixedSize(self.size, self.size)
            rob.show()
            return rob
        else:
            rob = ls.pop()
            rob.show()
            return rob

    def returnRobotLabel(self, rob, colorStr):
        """
        returns the robot
        :param rob: the robot
        :param colorStr: color as string
        :return:
        """
        self.pool[colorStr].append(rob)
        rob.hide()

class CrossPool:

    rp = rospkg.RosPack()

    def _getCrossFile(self, colorStr):
        """
        gets the path of the cross file
        :param colorStr: color as string
        :return:
        """
        return os.path.join(BallPool.rp.get_path('rqt_live_tool'), 'resource', 'ui_images', "cross-" + colorStr + "-big66x66.png")

    def __init__(self, frame, size=32):
        """

        :param frame: the frame
        :param size: the size
        """
        self.pool = {"blue": [], "green": [], "red": [], "yellow": []}
        self.frame = frame
        self.size = size

    def getCrossLabel(self, colorStr):
        """
        initiates the cross label
        :param colorStr: color as string
        :return:
        """
        ls = self.pool.get(colorStr)
        if ls == []:
            pxmap = QPixmap(self._getCrossFile(colorStr))
            crs = QLabel(parent=self.frame)
            crs.setPixmap(pxmap)
            crs.setScaledContents(True)
            crs.setFixedSize(self.size, self.size)
            crs.show()
            return crs
        else:
            crs = ls.pop()
            crs.show()
            return crs

    def returnCrossLabel(self, crs, colorStr):
        """
        returns the cross label
        :param crs: the cross
        :param colorStr: color as string
        :return:
        """
        self.pool[colorStr].append(crs)
        crs.hide()

class OpponentPool:

    rp = rospkg.RosPack()

    def _getOpponentFile(self, colorStr):
        """
        gets the path of the file
        :param colorStr: color as string
        :return:
        """
        #return os.path.join(BallPool.rp.get_path('rqt_live_tool'), 'resource', 'ui_images', "opponent_" + colorStr + "-big.png")
        return os.path.join(BallPool.rp.get_path('rqt_live_tool'), 'resource', 'ui_images', "new_icons", "opponent_" + colorStr + ".png")

    def __init__(self, frame, size=32):
        """

        :param frame: the frame
        :param size: the size
        """
        self.pool = {"blue": [], "green": [], "red": [], "yellow": [], "magenta": [], "cyan": []}
        self.frame = frame
        self.size = size

    def getOpponentLabel(self, colorStr):
        """
        initiates the opponent label
        :param colorStr: color as string
        :return:
        """
        ls = self.pool.get(colorStr)
        if ls == []:
            pxmap = QPixmap(self._getOpponentFile(colorStr))
            opp = QLabel(parent=self.frame)
            opp.setPixmap(pxmap)
            opp.setScaledContents(True)
            opp.setFixedSize(self.size, self.size)
            opp.show()
            return opp
        else:
            opp = ls.pop()
            opp.show()
            return opp

    def returnOpponentLabel(self, opp, colorStr):
        """
        returns the opponent label
        :param opp: the opponent
        :param colorStr: color as string
        :return:
        """
        self.pool[colorStr].append(opp)
        opp.hide()

class TeammatePool:

    rp = rospkg.RosPack()

    def _getTeammateFile(self, colorStr):
        """
        gets the path of the file
        :param colorStr: color as string
        :return:
        """
        #return os.path.join(BallPool.rp.get_path('rqt_live_tool'), 'resource', 'ui_images', "Rob1_" + colorStr + "-big.png")
        return os.path.join(BallPool.rp.get_path('rqt_live_tool'), 'resource', 'ui_images', "new_icons", "teammate_" + colorStr + ".png")

    def __init__(self, frame, size=32):
        """

        :param frame: frame
        :param size: size
        """
        self.pool ={"blue": [], "green": [], "red": [], "yellow": [], "cyan": [], "magenta": []}

        self.frame = frame
        self.size = size

    def getTeammateLabel(self, colorStr):
        """
        initiates the teammate label for the single field
        :param colorStr: color as string
        :return:
        """
        ls = self.pool.get(colorStr)
        if ls == []:
            pxmap = QPixmap(self._getTeammateFile(colorStr))
            mate = QLabel(parent=self.frame)
            mate.setPixmap(pxmap)
            mate.setScaledContents(True)
            mate.setFixedSize(self.size, self.size)
            mate.show()
            return mate
        else:
            mate = ls.pop()
            mate.show()
            return mate

    def returnTeammateLabel(self, mate, colorStr):
        """
        returns the teammate label
        :param mate: the teammate
        :param colorStr: color as string
        :return:
        """
        self.pool[colorStr].append(mate)
        mate.hide()

class CyanMagentaPool:

    rp = rospkg.RosPack()

    def _getTeammateFile(self, colorStr):
        """
        gets the teammate file
        :param colorStr: color as string
        :return:
        """
        return os.path.join(BallPool.rp.get_path('rqt_live_tool'), 'resource', 'ui_images', "Rob1_" + colorStr + "-big.png")

    def __init__(self, frame, size=32):
        """

        :param frame: frame
        :param size: size
        """
        self.pool ={"cyan": [], "magenta": []}

        self.frame = frame
        self.size = size

    def getTeammateLabel(self, colorStr):
        """
        initaties the teammate label for the quarter field
        :param colorStr: color as string
        :return:
        """
        ls = self.pool.get(colorStr)
        if ls == []:
            pxmap = QPixmap(self._getTeammateFile(colorStr))
            mate = QLabel(parent=self.frame)
            mate.setPixmap(pxmap)
            mate.setScaledContents(True)
            mate.setFixedSize(self.size, self.size)
            mate.show()
            return mate
        else:
            mate = ls.pop()
            mate.show()
            return mate

    def returnTeammateLabel(self, mate, colorStr):
        """
        returns the teammate label
        :param mate: the teammate
        :param colorStr: color as string
        :return: 
        """
        self.pool[colorStr].append(mate)
        mate.hide()

class UndefinedPool:

    rp = rospkg.RosPack()

    def _getUndefFile(self, colorStr=""):
        """
        gets the path for the file
        :param colorStr: color as string
        :return:
        """
        #return os.path.join(BallPool.rp.get_path('rqt_live_tool'), 'resource', 'ui_images', "Undef_grey-big33x67.png")
        return os.path.join(BallPool.rp.get_path('rqt_live_tool'), 'resource', 'ui_images', "new_icons", "undefined_" + colorStr + ".png")

    def __init__(self, frame, size=32):
        """

        :param frame: frame
        :param size: size
        """
        self.pool = {"blue": [], "green": [], "red": [], "yellow": [], "grey": []}
        self.frame = frame
        self.size = size

    def getUndefLabel(self, colorStr):
        """
        initiates the undefined label
        :param colorStr: color as string
        :return:
        """
        ls = self.pool.get(colorStr)
        if ls == []:
            pxmap = QPixmap(self._getUndefFile(colorStr))
            undf = QLabel(parent=self.frame)
            undf.setPixmap(pxmap)
            undf.setScaledContents(True)
            undf.setFixedSize(self.size, self.size)
            undf.show()
            return undf
        else:
            undf = ls.pop()
            undf.show()
            return undf

    def returnUndefLabel(self, undf, colorStr):
        """
        returns the undefined label
        :param undf: undefined obstacle
        :param colorStr: color as string
        :return:
        """
        self.pool[colorStr].append(undf)
        undf.hide()


class Arrowlabel(QWidget):

    def __init__(self, parent):
        super(Arrowlabel, self).__init__(parent)
        self.setFixedSize(80, 80)
        self.angleRobo = 0
        self.angleVel = 0
        self.color = QColor(111, 111, 111)



    def setLinearAngle(self, x, y):
        self.angleVel = self.radToDeg(self.legendAngle((x, y, 0), (1, 0, 0))) # winkel in rad
        size = self.len((x, y, 0)) * 2
        self.setFixedSize(size, size)
        self.update()


    def setRoboAngle(self, angle):
        self.angleRobo = angle
        self.update()



    def cross(self, a, b):
        return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

    def dot(self, a, b):
        return (a[0]*b[0]+ a[1]*b[1]+ a[2]*b[2])

    def len(self, a):
        return math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])

    def legendAngle(self, a, b):
        angle = 0
        sign = self.sign(self.cross(a, b)[2]) # z wert aus kreuzprodukt
        if sign == 0:
            sign = 1 # damit der vektor auch nach hinten zeigen kann

        if (self.len(a) * self.len(b)) != 0:
            angle = math.acos(self.dot(a,b) / (self.len(a) * self.len(b))) # immer positiv
        return angle * sign


    def sign(self, x):
        return math.copysign(1, x)


    def radToDeg(self, rads):
        return rads * 57.29578




    def paintEvent(self, event):
        painter = QPainter(self)
        #painter.begin(self)

        # puts the arrow in the middle
        painter.translate(self.width()/2, self.height()/2)
        painter.rotate(self.angleRobo + self.angleVel)


        line = QLineF(0, 0, self.width() / 2 - 3, 0)


        headSize = min(self.width() / 20, 4)
        points = QPolygonF()
        points.append(QPointF(self.width() / 2 - headSize * 2  , headSize))
        points.append(QPointF(self.width() / 2 - headSize * 2, - headSize))
        points.append(QPointF(self.width() / 2 -3, 0))



        pen = QPen(self.color, 2)
        painter.setPen(pen)
        brush = QBrush(self.color)
        painter.setBrush(brush)

        painter.drawLine(line)
        painter.drawConvexPolygon(points)
        #painter.end()


class AngularLabel(QWidget):
    def __init__(self, parent):
        super(AngularLabel, self).__init__(parent)
        self.colorGreen = QColor(66, 255, 100, 128)
        self.colorRed = QColor(255, 66, 100, 128)
        self.brushGreen = QBrush(self.colorGreen)
        self.brushRed = QBrush(self.colorRed)
        self.penGreen = QPen(self.brushGreen, 1)
        self.penRed = QPen(self.brushRed, 1)
        self.setFixedSize(100, 100)
        self.angle = 0
        self.velocity = 0

    def setAngles(self, startAbs, velocity=None):
        self.angle = startAbs * 16
        if velocity != None:
            self.velocity = velocity * 16
            self.update()


    def paintEvent(self, event):
        p = QPainter()
        p.begin(self)
        if self.velocity < 0:
            p.setBrush(self.brushGreen)
            p.setPen(self.penGreen)
        else:
            p.setBrush(self.brushRed)
            p.setPen(self.penRed)


        p.drawPie(0, 0, self.width(), self.height(), self.angle, self.velocity)

        p.end()
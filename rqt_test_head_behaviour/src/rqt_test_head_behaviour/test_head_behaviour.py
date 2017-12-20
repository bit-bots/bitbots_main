import os
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5 import Qt
from PyQt5 import QtCore
from PyQt5 import QtGui

from PyQt5.QtWidgets import QWidget

from humanoid_league_msgs.msg import BallRelative, BallsInImage, BallInImage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from bitbots_common.connector.connenctor import HeadConnector


class TestHeadBehaviour(Plugin):
    def __init__(self, context):
        super(TestHeadBehaviour, self).__init__(context)
        
        self.ball_x_default = 320
        self.ball_y_default = 160
        self.head_pan_default = 0   # Should be measured in degrees, message is radians
        self.head_tilt_default = 0  # Like pan

        # initialize the UI
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_test_head_behaviour'), 'resource', 'test_head_behaviour_widget.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('TestHeadBehaviourUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # ball_in_image GUI
        self.head_pan = self.head_pan_default
        self.head_tilt = self.head_tilt_default 
        self.head_refresh_edit()
        self.ball_x = self.ball_x_default
        self.ball_y = self.ball_y_default
        self._widget.clear_ball_button.clicked.connect(self.ball_clear_input)
        self._widget.publish_ball_in_image.clicked.connect(self.ball_in_image_pub)
        self._widget.ball_in_image_x.valueChanged.connect(self.ball_set_edit_values)
        self._widget.ball_in_image_y.valueChanged.connect(self.ball_set_edit_values)
        self.ball_refresh_edit()
        self.ball_refresh_sliders()
        self._widget.edit_x.textChanged.connect(self.ball_set_slider_values)
        self._widget.edit_y.textChanged.connect(self.ball_set_slider_values)
        # head_motor_goals GUI
        self._widget.clear_head_button.clicked.connect(self.head_clear_input)
#        self._widget.publish_head_motor_goals.clicked.connect(self.head_motor_goals_pub)
        self._widget.head_pan.valueChanged.connect(self.head_set_edit_values)
        self._widget.head_tilt.valueChanged.connect(self.head_set_edit_values)
        self._widget.edit_pan.textChanged.connect(self.head_set_slider_values)
        self._widget.edit_tilt.textChanged.connect(self.head_set_slider_values)
        self.head_refresh_edit()
        self.head_refresh_sliders()

        self.ball_in_image_publisher = rospy.Publisher("ball_in_image", BallsInImage, queue_size = 1)
        rospy.Subscriber("head_motor_goals", JointTrajectory, self.head_motor_goals_cb, queue_size=1)
        context.add_widget(self._widget)

        # set connector.vision.ball_seen to a recent time, so that the ball is tracked
        # maybe update it to the current time when the ball in image message is published

    def head_motor_goals_cb(self, msg):
        self.head_pan = msg.points[0].positions[0]
        self.head_tilt = msg.points[0].positions[1]
        self.head_refresh_edit()
        self.head_refresh_sliders()

    # ball_in_image
    def ball_set_edit_values(self):
        self.ball_x = self._widget.ball_in_image_x.value() 
        self.ball_y = self._widget.ball_in_image_y.value()
        self.ball_refresh_edit()

    def ball_refresh_edit(self):
        self._widget.edit_x.setText(str(self.ball_x))
        self._widget.edit_y.setText(str(self.ball_y))

    def ball_set_slider_values(self):
        self.ball_x = int(self._widget.edit_x.text())
        self.ball_y = int(self._widget.edit_y.text())
        self.ball_refresh_sliders()

    def ball_refresh_sliders(self):
        self._widget.ball_in_image_x.setValue(self.ball_x)
        self._widget.ball_in_image_y.setValue(self.ball_y)

    def ball_clear_input(self):
        self.ball_x = self.ball_x_default
        self.ball_y = self.ball_y_default
        self.ball_refresh_edit()
        self.ball_refresh_sliders()

    def ball_in_image_pub(self):
        # TODO: add connector.vision.ball_seen update
        # connector.vision.
        ball_in_image_msg = BallsInImage()
        can = BallInImage()
        can.center.x = self.ball_x
        can.center.y = self.ball_y
        can.diameter = 100
        can.confidence = 1
        ball_in_image_msg.candidates.append(can)
        self.ball_in_image_publisher.publish(ball_in_image_msg)
        rospy.loginfo("Published ball_in_image")


    # head_motor_goals

    def head_set_edit_values(self):
        self.head_pan = self._widget.head_pan.value()/self.pan_factor
        self.head_tilt = self._widget.head_tilt.value()/self.tilt_factor
        self.head_refresh_edit()

    def head_refresh_edit(self):
        self._widget.edit_pan.setText(str(self.head_pan))
        self._widget.edit_tilt.setText(str(self.head_tilt))

    def head_set_slider_values(self):
        rospy.logwarn("Text:" + self._widget.edit_tilt.text())
        rospy.logwarn("Wert:" + str(self.head_tilt))
        self.head_pan = float(self._widget.edit_pan.text())*self.pan_factor
        self.head_tilt = float(self._widget.edit_tilt.text())*self.tilt_factor
        self.head_refresh_sliders()

    def head_refresh_sliders(self):
        self._widget.head_pan.setValue(self.head_pan)
        self._widget.head_tilt.setValue(self.head_tilt) 

    def head_clear_input(self):
        self.head_pan = 0
        self.head_tilt = 0
        self.head_refresh_edit()
        self.head_refresh_sliders()

    def head_motor_goals_pub(self):
        head_motor_goals_msg = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = [self.head_pan, self.head_tilt]
        head_motor_goals_msg.points.append(point)
        self.head_motor_goals_publisher.publish(head_motor_goals_msg)
        rospy.loginfo("Published head_motor_goals")

    # ball_relative
    def ball_relative_cb(self, msg):
        # TODO: insert right/left
        x = msg.ball_relative.x
        y = msg.ball_relative.y
        t = "x = "+str(x)+", y = "+str(y)
        self._widget.ball_relative_out.setText(t)

    def shutdown_plugin(self):
        # TODO: unregister publishers and subscribers
        pass
        

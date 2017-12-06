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


class TestHeadBehaviour(Plugin):
    def __init__(self, context):
        super(TestHeadBehaviour, self).__init__(context)

        self.head_pan = 0.0
        self.head_tilt = 0.0
        self.ball_x = 0
        self.ball_y = 0

        # initialize the UI
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_test_head_behaviour'), 'resource', 'test_head_behaviour_widget.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('TestHeadBehaviourUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # ball_in_image GUI
        self._widget.clear_ball_button.clicked.connect(self.ball_clear_input)
        self._widget.publish_ball_in_image.clicked.connect(self.ball_in_image_pub)
        self._widget.ball_in_image_x.valueChanged.connect(self.ball_set_edit_values)
        self._widget.ball_in_image_y.valueChanged.connect(self.ball_set_edit_values)
        self._widget.edit_x.setText("0")
        self._widget.edit_y.setText("0")
        self._widget.edit_x.textChanged.connect(self.ball_set_slider_values)
        self._widget.edit_y.textChanged.connect(self.ball_set_slider_values)
        # head_motor_goals GUI
        self._widget.clear_head_button.clicked.connect(self.head_clear_input)
        self._widget.publish_head_motor_goals.clicked.connect(self.head_motor_goals_pub)
        self._widget.head_pan.valueChanged.connect(self.head_set_edit_values)
        self._widget.head_tilt.valueChanged.connect(self.head_set_edit_values)
        self._widget.edit_pan.setText("0")
        self._widget.edit_tilt.setText("0")
        self._widget.edit_pan.textChanged.connect(self.head_set_slider_values)
        self._widget.edit_tilt.textChanged.connect(self.head_set_slider_values)

        self.ball_in_image_publisher = rospy.Publisher("ball_in_image", BallsInImage, queue_size = 1)
        self.head_motor_goals_publisher = rospy.Publisher("head_motor_goals", JointTrajectory, queue_size = 1)
        rospy.Subscriber("/ball_relative", BallRelative, self.ball_relative_cb, queue_size=1)
        rospy.Subscriber("/head_motor_goals", JointTrajectory, self.head_motor_goals_cb, queue_size=1)
        context.add_widget(self._widget)

    def head_motor_goals_cb(self, msg):
        pan = msg.points[0].positions[0]
        tilt = msg.points[0].positions[1]
        self._widget.edit_pan.setText(str(pan))
        self._widget.edit_tilt.setText(str(pan))
        self._widget.head_pan.setValue(int(pan*1000))
        self._widget.head_tilt.setValue(int(tilt*1000))

    # ball_in_image
    def ball_set_edit_values(self):
        self.ball_x = self._widget.ball_in_image_x.value() 
        self.ball_y = self._widget.ball_in_image_y.value() 
       
        self._widget.edit_x.setText(str(self.ball_x))
        self._widget.edit_y.setText(str(self.ball_y))

    def ball_set_slider_values(self):
#        self.ball_x = self._widget.ed
        self._widget.ball_in_image_x.setValue(int(self._widget.edit_x.text()))
        self._widget.ball_in_image_y.setValue(int(self._widget.edit_y.text()))

    def ball_clear_input(self):
        self._widget.ball_in_image_x.setValue(320)
        self._widget.ball_in_image_y.setValue(160)
        self._widget.edit_x.setText("320")
        self._widget.edit_y.setText("160")

    def ball_in_image_pub(self):
        # TODO: implement diameter
        ball_in_image_msg = BallsInImage()
        can = BallInImage()
        can.center.x = self._widget.ball_in_image_x.value()
        can.center.y = self._widget.ball_in_image_y.value()
        can.diameter = 100
        can.confidence = 1
        ball_in_image_msg.candidates.append(can)
        self.ball_in_image_publisher.publish(ball_in_image_msg)
        rospy.loginfo("Published ball_in_image")


    # head_motor_goals
    def head_set_edit_values(self):
        self._widget.edit_pan.setText(str(self._widget.head_pan.value()/1000.0))
        self._widget.edit_tilt.setText(str(self._widget.head_tilt.value()/1000.0))

    def head_set_slider_values(self):
        self._widget.head_pan.setValue(float(self._widget.edit_pan.text())*1000)
        self._widget.head_tilt.setValue(float(self._widget.edit_tilt.text())*1000)

    def head_clear_input(self):
        self._widget.head_pan.setValue(0)
        self._widget.head_tilt.setValue(0)
        self._widget.edit_pan.setText("0")
        self._widget.edit_tilt.setText("0")

    def head_motor_goals_pub(self):
        head_motor_goals_msg = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = [self._widget.ball_in_image_x.value()/1000.0, self._widget.ball_in_image_y.value()/1000.0]
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
        self.stop_publishing("ball_in_image")
        self.stop_publishing("head_motot_goals")
        

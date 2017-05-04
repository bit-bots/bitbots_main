#!/usr/bin/env python3
import rospkg
import rospy

from python_qt_binding import loadUi
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget
from sensor_msgs import JointState

import os


class RecordUI(Plugin):
    def __init__(self, context):
        super(RecordUI, self).__init__(context)
        self.setObjectName('Record Animation')

        rospy.Subscriber("/joint_states", JointState, self.state_update, queue_size=100)

        self._widget = QWidget()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('bitbots_recordui_rqt'), 'resource', 'RecordUI.ui')
        loadUi(ui_file, self._widget, {})
        
        #self._widget.record_frame_button.pressed.connect(self.record_frame)

        context.add_widget(self._widget)

    def record_frame(self):
        return 0;

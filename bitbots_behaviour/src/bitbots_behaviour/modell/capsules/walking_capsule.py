# -*- coding:utf-8 -*-
"""
WalkingCapsule
^^^^^^^^^^^^^^

.. moduleauthor:: sheepy <sheepy@informatik.uni-hamburg.de>

History:
* 4/16/14: Created (sheepy)

"""
import copy

from bitbots.debug import Scope
from bitbots.modules.abstract.abstract_module import debug_m
from bitbots.util import get_config

import rospy
from nav_msgs.msg import Odometry


class WalkingCapsule():

    def __init__(self):

        config = get_config()["Behaviour"]["Common"]["Walking"]
        self.odometry_data = Odometry()
        self.publisher = None


    def start_walking_plain(self, forward, angular, sideward=0):


    def start_walking(self, forward_key=ZERO, angular_key=ZERO, sidewards_key=ZERO):
        # Assert that the key is valid


    def set_angular_direct(self, value):


    def get_walking_correction_values(self):
        """This method is delivers three values that are applied additionally
            to every call to the walking engine - that can be used for correction of single robots """

    def stop_walking(self):
        """ This method stops the walking - note that it could take some time until the robot is standing """


    def is_walking(self):
        """ This method returns True if the walking is actually not running """

    def walking_callbacl(self, od:Odometry):
        self.odometry_data = copy.copy(od)

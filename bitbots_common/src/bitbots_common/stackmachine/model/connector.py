# -*- coding:utf-8 -*-
"""
Connector
^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from bitbots_common.stackmachine.model.capsules.animation_capsule import AnimationCapsule
from bitbots_common.stackmachine.model.capsules.blackboard_capsule import BlackboardCapsule
from bitbots_common.stackmachine.model.capsules.game_status_capsule import GameStatusCapsule
from bitbots_common.stackmachine.model.capsules.vision_capsule import VisionCapsule
from bitbots_common.stackmachine.model.capsules.walking_capsule import WalkingCapsule
from bitbots_common.stackmachine.model.capsules.world_model_capsule import WorldModelCapsule

import rospy
from bitbots_common.stackmachine.model.capsules.team_data_capsule import TeamDataCapsule


class Connector:

    def __init__(self):
        self.vision = VisionCapsule()
        self.world_model = WorldModelCapsule()
        self.blackboard = BlackboardCapsule()

        self.gamestate = GameStatusCapsule()
        self.walking = WalkingCapsule()

        self.team_data = TeamDataCapsule()
        self.animation = AnimationCapsule()

        self.speaker: rospy.Publisher

        #self._robot = Robot()
        #self._kinematic_task = KinematicTask(self._robot)



    def get_pose(self):
        return self.data["Pose"]

    def set_pose(self, pose):
        self.data["Ipc"].update(pose)

    def get_ipc(self):
        return self.data["Ipc"]



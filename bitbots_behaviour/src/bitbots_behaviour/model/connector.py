# -*- coding:utf-8 -*-
"""
Connector
^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
import rospy
from model.capsules.animation_capsule import AnimationCapsule
from model.capsules.blackboard_capsule import BlackboardCapsule
from model.capsules.game_status_capsule import GameStatusCapsule
from model.capsules.world_model_capsule import WorldModelCapsule
from model.capsules.vision_capsule import VisionCapsule
from model.capsules.team_data_capsule import TeamDataCapsule
from model.capsules.walking_capsule import WalkingCapsule


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



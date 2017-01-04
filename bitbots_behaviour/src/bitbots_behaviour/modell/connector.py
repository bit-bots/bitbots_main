# -*- coding:utf-8 -*-
"""
Connector
^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
import rospy
from modell.capsules.animation_capsule import AnimationCapsule
from modell.capsules.blackboard_capsule import BlackboardCapsule
from modell.capsules.filtered_vision_capsule import FilteredVisionCapsule
from modell.capsules.game_status_capsule import GameStatusCapsule
from modell.capsules.world_model_capsule import WorldModelCapsule
from modell.capsules.vision_capsule import VisionCapsule
from modell.capsules.team_data_capsule import TeamDataCapsule
from modell.capsules.walking_capsule import WalkingCapsule


class Connector:

    def __init__(self):
        self.vision = VisionCapsule()
        self.world_model = WorldModelCapsule()
        self.blackboard = BlackboardCapsule()

        self.gamestate = GameStatusCapsule()
        #self.walking = WalkingCapsule()

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



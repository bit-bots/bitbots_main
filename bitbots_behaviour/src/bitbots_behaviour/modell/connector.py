# -*- coding:utf-8 -*-
"""
Connector
^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""

from modell.capsules.animation_capsule import AnimationCapsule
from modell.capsules.blackboard_capsule import BlackboardCapsule
from modell.capsules.filtered_vision_capsule import FilteredVisionCapsule
from modell.capsules.game_status_capsule import GameStatusCapsule
from modell.capsules.world_model_capsule import WorldModelCapsule
from modell.capsules.raw_vision_capsule import RawVisionCapsule
from modell.capsules.team_data_capsule import TeamDataCapsule
from modell.capsules.walking_capsule import WalkingCapsule


class Connector:

    def __init__(self):

        self.gamestate = GameStatusCapsule()
        self.walking = WalkingCapsule()
        self.blackboard = BlackboardCapsule()
        self.team_data = TeamDataCapsule()
        self.animation = AnimationCapsule()
        self.raw_vision = RawVisionCapsule()
        self.filtered_vision = FilteredVisionCapsule()
        self.world_model = WorldModelCapsule()
        self.behaviour = BehaviourCapsule()
        self.data = dict()

        self._robot = Robot()
        self._kinematic_task = KinematicTask(self._robot)

    def __getitem__(self, item):
        """
        Fallbacklösung, falls noch versucht wird auf das dataarray zuzugreifen
        """
        assert item in self.data
        return self.data[item]

    def is_key_in_data(self, key):
        return key in self.data

    def get_pose(self):
        return self.data["Pose"]

    def set_pose(self, pose):
        self.data["Ipc"].update(pose)

    def get_ipc(self):
        return self.data["Ipc"]

    def set_duty(self, duty):
        self.data["Duty"] = duty
        role = ROLE_OTHER
        if duty == "Goalie":
            role = ROLE_GOALIE
        if duty in ("Fieldie", "TeamPlayer"):
            role = ROLE_SUPPORTER
        self.data[DATA_KEY_ROLE] = role

    def get_duty(self):
        return self.data.get("Duty", False)

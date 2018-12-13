#!/usr/bin/env python2.7

import rospy
from bitbots_blackboard.abstract_connector import AbstractConnector
from bitbots_blackboard.capsules.animation_capsule import AnimationCapsule
from bitbots_blackboard.capsules.blackboard_capsule import BlackboardCapsule
from bitbots_blackboard.capsules.game_status_capsule import GameStatusCapsule
from bitbots_blackboard.capsules.team_data_capsule import TeamDataCapsule
from bitbots_blackboard.capsules.world_model_capsule import WorldModelCapsule
from bitbots_blackboard.capsules.head_capsule_old import HeadCapsule
from bitbots_blackboard.capsules.speaker_capsule import SpeakerCapsule
from bitbots_blackboard.capsules.pathfinding_capsule import PathfindingCapsule


       
class BodyConnector(AbstractConnector):
    def __init__(self):
        super(BodyConnector, self).__init__()
        self.config = rospy.get_param("Behaviour")
        self.blackboard = BlackboardCapsule()
        self.gamestate = GameStatusCapsule()
        self.animation = AnimationCapsule()
        self.speaker = SpeakerCapsule()
        self.pathfinding = PathfindingCapsule()
        self.world_model = WorldModelCapsule(self.config)
        self.team_data = TeamDataCapsule()


class HeadConnector(AbstractConnector):
    def __init__(self):
        super(HeadConnector, self).__init__()
        self.config = rospy.get_param("Behaviour")
        self.head = HeadCapsule()
        self.world_model = WorldModelCapsule(self.config)
        self.team_data = TeamDataCapsule()

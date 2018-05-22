#!/usr/bin/env python2.7

import rospy
from bitbots_connector.capsules.animation_capsule import AnimationCapsule
from bitbots_connector.capsules.blackboard_capsule import BlackboardCapsule
from bitbots_connector.capsules.game_status_capsule import GameStatusCapsule
from bitbots_connector.capsules.team_data_capsule import TeamDataCapsule
from bitbots_connector.capsules.world_model_capsule import WorldModelCapsule
from bitbots_connector.capsules.head_capsule import HeadCapsule
from bitbots_connector.capsules.speaker_capsule import SpeakerCapsule


class AbstractConnector(object):
    def __init__(self):
        self.world_model = WorldModelCapsule()
        self.team_data = TeamDataCapsule()

        self.config = None  # type: dict


class BodyConnector(AbstractConnector):
    def __init__(self):
        super(BodyConnector, self).__init__()
        self.blackboard = BlackboardCapsule()
        self.gamestate = GameStatusCapsule()
        self.animation = AnimationCapsule()
        self.speaker = SpeakerCapsule()
        
        self.pathfinding_publisher = None  # type: rospy.Publisher


class HeadConnector(AbstractConnector):
    def __init__(self):
        super(HeadConnector, self).__init__()
        self.head = HeadCapsule()

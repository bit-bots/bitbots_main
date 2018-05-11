#!/usr/bin/env python2.7

import rospy
from bitbots_common.connector.capsules.animation_capsule import AnimationCapsule
from bitbots_common.connector.capsules.blackboard_capsule import BlackboardCapsule
from bitbots_common.connector.capsules.game_status_capsule import GameStatusCapsule
from bitbots_common.connector.capsules.team_data_capsule import TeamDataCapsule
from bitbots_common.connector.capsules.walking_capsule import WalkingCapsule
from bitbots_common.connector.capsules.world_model_capsule import WorldModelCapsule
from bitbots_common.connector.capsules.head_capsule import HeadCapsule
from bitbots_common.connector.capsules.personal_model_capsule import PersonalModelCapsule
from bitbots_common.connector.capsules.speaker_capsule import SpeakerCapsule


class AbstractConnector(object):
    def __init__(self):
        self.personal_model = PersonalModelCapsule()
        self.world_model = WorldModelCapsule()
        self.team_data = TeamDataCapsule()

        self.config = None  # type: dict


class BodyConnector(AbstractConnector):
    def __init__(self):
        super(BodyConnector, self).__init__()
        self.blackboard = BlackboardCapsule()
        self.gamestate = GameStatusCapsule()
        self.walking = WalkingCapsule()
        self.animation = AnimationCapsule()
        self.speaker = SpeakerCapsule()
        
        self.pathfinding_publisher = None  # type: rospy.Publisher


class HeadConnector(AbstractConnector):
    def __init__(self):
        super(HeadConnector, self).__init__()
        self.head = HeadCapsule()

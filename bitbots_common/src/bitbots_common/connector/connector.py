#!/usr/bin/env python3

import rospy
from bitbots_common.connector.capsules.animation_capsule import AnimationCapsule
from bitbots_common.connector.capsules.blackboard_capsule import BlackboardCapsule
from bitbots_common.connector.capsules.game_status_capsule import GameStatusCapsule
from bitbots_common.connector.capsules.team_data_capsule import TeamDataCapsule
from bitbots_common.connector.capsules.walking_capsule import WalkingCapsule
from bitbots_common.connector.capsules.world_model_capsule import WorldModelCapsule
from bitbots_common.connector.capsules.head_capsule import HeadCapsule
from bitbots_common.connector.capsules.vision_capsule import VisionCapsule


class AbstractConnector:
    def __init__(self):
        self.vision = VisionCapsule()
        self.world_model = WorldModelCapsule()
        self.team_data = TeamDataCapsule()

        self.config = None  # type: dict


class BodyConnector(AbstractConnector):
    def __init__(self):
        super().__init__()
        self.blackboard = BlackboardCapsule()
        self.gamestate = GameStatusCapsule()
        self.walking = WalkingCapsule()
        self.animation = AnimationCapsule()

        self.head_pub = None  # type: rospy.Publisher
        self.speaker = None  # type: rospy.Publisher


class HeadConnector(AbstractConnector):
    def __init__(self):
        super().__init__()
        self.head = HeadCapsule()

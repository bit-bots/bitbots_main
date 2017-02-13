"""
Connector
^^^^^^^^^

.. moduleauthor:: Martin Poppinga <robocup@poppinga.xyz>

"""
import rospy
from model.capsules.animation_capsule import AnimationCapsule
from model.capsules.blackboard_capsule import BlackboardCapsule
from model.capsules.game_status_capsule import GameStatusCapsule
from model.capsules.team_data_capsule import TeamDataCapsule
from model.capsules.vision_capsule import VisionCapsule
from model.capsules.walking_capsule import WalkingCapsule
from model.capsules.world_model_capsule import WorldModelCapsule


class Connector:
    def __init__(self):
        self.vision = VisionCapsule()
        self.world_model = WorldModelCapsule()
        self.blackboard = BlackboardCapsule()
        self.gamestate = GameStatusCapsule()
        self.walking = WalkingCapsule()
        self.team_data = TeamDataCapsule()
        self.animation = AnimationCapsule()

        self.speaker = None  # type: rospy.Publisher

        self.config = None  # type: dict

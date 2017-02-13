"""
Connector
^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
import rospy
from stackmachine.model.capsules.animation_capsule import AnimationCapsule
from stackmachine.model.capsules.blackboard_capsule import BlackboardCapsule
from stackmachine.model.capsules.game_status_capsule import GameStatusCapsule
from stackmachine.model.capsules.team_data_capsule import TeamDataCapsule
from stackmachine.model.capsules.vision_capsule import VisionCapsule
from stackmachine.model.capsules.walking_capsule import WalkingCapsule
from stackmachine.model.capsules.world_model_capsule import WorldModelCapsule

from bitbots_head_behaviour.src.bitbots_head_behaviour.head_connector import HeadCapsule


class Connector:
    def __init__(self):
        self.vision = VisionCapsule()
        self.world_model = WorldModelCapsule()
        self.blackboard = BlackboardCapsule()
        self.gamestate = GameStatusCapsule()
        self.head = HeadCapsule()
        self.walking = WalkingCapsule()
        self.team_data = TeamDataCapsule()
        self.animation = AnimationCapsule()

        self.speaker = None  # type: rospy.Publisher

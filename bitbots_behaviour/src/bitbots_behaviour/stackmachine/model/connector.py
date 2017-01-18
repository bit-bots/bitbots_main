"""
Connector
^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
import rospy
from stackmachine.model import AnimationCapsule
from stackmachine.model import BlackboardCapsule
from stackmachine.model import GameStatusCapsule
from stackmachine.model import TeamDataCapsule
from stackmachine.model import VisionCapsule
from stackmachine.model import WalkingCapsule
from stackmachine.model import WorldModelCapsule


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

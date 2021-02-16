import rospy
from bio_ik_msgs.srv import GetIK
from bitbots_blackboard.capsules.animation_capsule import AnimationCapsule
from bitbots_blackboard.capsules.blackboard_capsule import BlackboardCapsule
from bitbots_blackboard.capsules.game_status_capsule import GameStatusCapsule
from bitbots_blackboard.capsules.head_capsule import HeadCapsule
from bitbots_blackboard.capsules.kick_capsule import KickCapsule
from bitbots_blackboard.capsules.pathfinding_capsule import PathfindingCapsule
from bitbots_blackboard.capsules.team_data_capsule import TeamDataCapsule
from bitbots_blackboard.capsules.world_model_capsule import WorldModelCapsule


class BodyBlackboard:
    def __init__(self):

        self.config = rospy.get_param("behavior/body")
        self.blackboard = BlackboardCapsule()
        self.gamestate = GameStatusCapsule()
        self.animation = AnimationCapsule()
        self.kick = KickCapsule(self)
        self.pathfinding = PathfindingCapsule()
        self.world_model = WorldModelCapsule()
        self.team_data = TeamDataCapsule()


class HeadBlackboard:
    def __init__(self):
        self.config = rospy.get_param("behavior/head")
        self.head_capsule = HeadCapsule(self)
        self.world_model = WorldModelCapsule()
        rospy.wait_for_service('bio_ik/get_bio_ik')
        self.bio_ik = rospy.ServiceProxy('bio_ik/get_bio_ik', GetIK)

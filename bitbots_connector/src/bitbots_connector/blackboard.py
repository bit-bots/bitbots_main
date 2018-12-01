import rospy
from bio_ik_msgs.srv import GetIK
from bitbots_connector.capsules.animation_capsule import AnimationCapsule
from bitbots_connector.capsules.blackboard_capsule import BlackboardCapsule
from bitbots_connector.capsules.game_status_capsule import GameStatusCapsule
from bitbots_connector.capsules.head_capsule import HeadCapsule
from bitbots_connector.capsules.pathfinding_capsule import PathfindingCapsule
from bitbots_connector.capsules.team_data_capsule import TeamDataCapsule
from bitbots_connector.capsules.world_model_capsule import WorldModelCapsule


class BodyBlackboard:
    def __init__(self):
        # TODO this will not stay here, don't worry
        # it is just that I don't know where this will be placed later
        self.field_width = 6
        self.field_length = 9

        self.config = rospy.get_param("behavior/body")
        self.blackboard = BlackboardCapsule()
        self.gamestate = GameStatusCapsule()
        self.animation = AnimationCapsule()
        self.pathfinding = PathfindingCapsule()
        self.world_model = WorldModelCapsule(self.field_length, self.field_width)
        self.team_data = TeamDataCapsule()


class HeadBlackboard:
    def __init__(self):
        self.config = rospy.get_param("behavior/head")
        self.head_capsule = HeadCapsule(self)
        self.world_model = WorldModelCapsule(6, 9)  # TODO move this to a good place
        rospy.wait_for_service('/bio_ik/get_bio_ik')
        self.bio_ik = rospy.ServiceProxy('/bio_ik/get_bio_ik', GetIK)

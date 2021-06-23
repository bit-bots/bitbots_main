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

import actionlib
from humanoid_league_msgs.msg import PlayAnimationAction


class BodyBlackboard:
    def __init__(self):

        self.config = rospy.get_param("behavior/body")
        self.base_footprint_frame = rospy.get_param("~base_footprint_frame", "base_footprint")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.blackboard = BlackboardCapsule()
        self.gamestate = GameStatusCapsule()
        self.animation = AnimationCapsule()
        self.kick = KickCapsule(self)
        self.world_model = WorldModelCapsule(self)
        self.pathfinding = PathfindingCapsule(self)
        self.world_model = WorldModelCapsule(self)
        self.team_data = TeamDataCapsule()
        # animations
        self.animation_action_client = actionlib.SimpleActionClient('animation', PlayAnimationAction)
        self.goalie_arms_animation = rospy.get_param("Animations/Goalie/goalieArms")
        self.goalie_falling_right_animation = rospy.get_param("Animations/Goalie/fallRight")
        self.goalie_falling_left_animation = rospy.get_param("Animations/Goalie/fallLeft")
        self.goalie_falling_center_animation = rospy.get_param("Animations/Goalie/fallCenter")
        self.cheering_animation = rospy.get_param("Animations/Misc/cheering")
        self.init_animation = rospy.get_param("Animations/Misc/init")

        self.dynup_action_client = None
        self.dynup_cancel_pub = None  # type: rospy.Publisher
        self.hcm_deactivate_pub = None  # type: rospy.Publisher

class HeadBlackboard:
    def __init__(self):
        self.config = rospy.get_param("behavior/head")
        self.head_capsule = HeadCapsule(self)
        self.world_model = WorldModelCapsule(self)
        rospy.wait_for_service('bio_ik/get_bio_ik')
        self.bio_ik = rospy.ServiceProxy('bio_ik/get_bio_ik', GetIK)

import rospy
from bitbots_hcm.hcm_dsd.actions.play_animation import PlayAnimationDynup


class GetWalkready(PlayAnimationDynup):
    def __init__(self, blackboard, dsd, parameters=None):
        rospy.logerr("dynup body action")
        super(GetWalkready, self).__init__(blackboard, dsd, parameters={'initial': True, 'direction': 'walkready'})

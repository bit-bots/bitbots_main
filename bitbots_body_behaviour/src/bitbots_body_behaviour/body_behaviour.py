#!/usr/bin/env python3

"""
BehaviourModule
^^^^^^^^^^^^^^^
.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Starts the body behaviour
"""

import actionlib
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D
from humanoid_league_msgs.msg import BallRelative, GameState, Speak, HeadMode, Strategy, TeamData, PlayAnimationAction

from bitbots_connector.blackboard import BodyBlackboard
from dsd import dsd


def run(self):
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        self.update()
        rate.sleep()


if __name__ == "__main__":
    D = dsd.DSD(BodyBlackboard())

    D.blackboard.speaker.speaker = rospy.Publisher("speak", Speak, queue_size=3)
    D.blackboard.team_data.strategy_sender = rospy.Publisher("strategy", Strategy, queue_size=2)
    D.blackboard.blackboard.head_pub = rospy.Publisher("head_duty", HeadMode, queue_size=10)
    D.blackboard.pathfinding.pathfinding_simple_pub = rospy.Publisher('bitbots_pathfinding/relative_goal', Pose2D,
                                                                      queue_size=10)

    D.register_actions("actions", recursive=True)
    D.register_decisions("decisions", recursive=True)

    D.load_behavior("main.dsd")

    rospy.init_node("Bodybehavior")

    rospy.Subscriber("ball_relative", BallRelative, D.blackboard.world_model.ball_callback)
    rospy.Subscriber("gamestate", GameState, D.blackboard.gamestate.gamestate_callback)
    rospy.Subscriber("team_data", TeamData, D.blackboard.team_data.team_data_callback)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, D.blackboard.world_model.position_callback)

    D.blackboard.animation.server = actionlib.SimpleActionClient("bitbots_animation", PlayAnimationAction)

    # for k, v in D.tree.items():
    #    print(k, v)

    # print(D.tree)
    # print(json.dumps(D.tree,  indent=4))
    run(D)

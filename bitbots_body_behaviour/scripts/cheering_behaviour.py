#!/usr/bin/env python3
import actionlib
import rospy
from humanoid_league_msgs.msg import PlayAnimationGoal, PlayAnimationAction, GameState


class CheeringBehaviour:
    def __init__(self):
        rospy.init_node('cheering_behvaiour')

        self.opponent_goals = 0
        self.animation_running = False

        rospy.Subscriber('/gamestate', GameState, self.gamestate_callback, queue_size=10)
        self.animation_client = actionlib.SimpleActionClient('animation', PlayAnimationAction)

        rospy.spin()

    def gamestate_callback(self, msg: GameState):
        if msg.rivalScore > self.opponent_goals and not self.animation_running:
            goal = PlayAnimationGoal()
            goal.animation = 'cheering'  # TODO
            goal.hcm = False
            self.animation_client.send_goal(goal, done_cb=self.animation_done)
            self.animation_running = True

    def animation_done(self, arg1, arg2):
        self.animation_running = False


if __name__ == '__main__':
    CheeringBehaviour()

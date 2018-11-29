#!/usr/bin/env python3
import actionlib
import rospy
from humanoid_league_msgs.msg import PlayAnimationGoal, PlayAnimationAction, GameState
from std_msgs.msg import Bool


class PenaltyBehaviour:
    def __init__(self):
        rospy.init_node('penalty_behaviour')
        self.connected = False
        self.penalty_kick_allowed = False
        self.animation_running = False
        self.button_pressed = False
        self.pressed_time = 0
        self.kick_animation = 'penalty_kick_left_kamikaze_higher6_actually_working_standing_end'
        self.last_animation = None
        self.animation_client = actionlib.SimpleActionClient('animation', PlayAnimationAction)

        rospy.Subscriber('wifi_connected', Bool, self.wifi_callback, queue_size=1)
        rospy.Subscriber('shoot_button', Bool, self.button_callback, queue_size=1)
        rospy.Subscriber('gamestate', GameState, self.gamestate_callback, queue_size=1)

        rospy.sleep(1)
        self.play_animation('walkready')

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if ((self.connected and self.penalty_kick_allowed and not self.animation_running or
                    not self.connected and self.button_pressed and rospy.get_time() - self.pressed_time > 10)
                    and not self.animation_running):
                rospy.loginfo('Playing kick animation')
                self.play_animation(self.kick_animation)
                self.button_pressed = False
            rate.sleep()

    def wifi_callback(self, msg: Bool):
        self.connected = msg.data

    def gamestate_callback(self, msg: GameState):
        self.penalty_kick_allowed = msg.allowedToMove and msg.gameState == GameState.GAMESTATE_PLAYING

    def button_callback(self, msg: Bool):
        rospy.loginfo('Button pressed, kick in 10 seconds')
        self.pressed_time = rospy.get_time()
        self.button_pressed = msg.data

    def play_animation(self, name):
        self.last_animation = name
        goal = PlayAnimationGoal()
        goal.animation = name
        goal.hcm = False
        self.animation_client.send_goal(goal, done_cb=self.animation_done)
        self.animation_running = True

    def animation_done(self, arg1, arg2):
        self.animation_running = False
        if self.last_animation == self.kick_animation:
            rospy.sleep(10)
            self.play_animation('walkready')


if __name__ == '__main__':
    PenaltyBehaviour()

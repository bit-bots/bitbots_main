#!/usr/bin/env python3
import rospy

from humanoid_league_msgs.msg import GameState, RobotControlState
from geometry_msgs.msg import PoseWithCovarianceStamped

from dynamic_stack_decider.dsd import DSD
from bitbots_localization.localization_dsd.localization_blackboard import LocalizationBlackboard
import os


class LocalizationHandler(object):
    def __init__(self):

        # --- Initialize Node ---
        log_level = rospy.DEBUG if rospy.get_param("debug_active", False) else rospy.INFO
        rospy.init_node('localization_handler', log_level=log_level, anonymous=False)
        rospy.sleep(0.1)  # Otherwise messages will get lost, bc the init is not finished
        rospy.loginfo("Starting localization handler")

        # stack machine
        self.blackboard = LocalizationBlackboard()
        dirname = os.path.dirname(os.path.realpath(__file__)) + "/localization_dsd"
        rospy.loginfo(dirname)
        self.dsd = DSD(self.blackboard, "/debug/dsd/localization")
        self.dsd.register_actions(os.path.join(dirname, 'actions'))
        self.dsd.register_decisions(os.path.join(dirname, 'decisions'))
        self.dsd.load_behavior(os.path.join(dirname, 'localization.dsd'))

        rospy.Subscriber("bitbots_localization/pose_with_covariance", PoseWithCovarianceStamped, self._callback_pose, queue_size=1)
        rospy.Subscriber("gamestate", GameState, self._callback_game_state, queue_size=1)
        rospy.Subscriber("robot_state", RobotControlState, self._callback_robot_control_state, queue_size=1)

        self.main_loop()

        rospy.spin()

    def _callback_pose(self, msg):
        self.blackboard.last_pose_update_time = msg.header.stamp
        self.blackboard.poseX = msg.pose.pose.position.x
        self.blackboard.poseY = msg.pose.pose.position.y
        self.blackboard.orientation = msg.pose.pose.orientation
        self.blackboard.covariance = msg.pose.covariance

    def _callback_game_state(self, msg):
        self.blackboard.game_state_received = True
        self.blackboard.game_state = msg.gameState
        self.blackboard.secondary_state = msg.secondaryState
        self.blackboard.first_half = msg.firstHalf
        self.blackboard.has_kickoff = msg.hasKickOff
        self.blackboard.penalized = msg.penalized
        self.blackboard.secondsTillUnpenalized = msg.secondsTillUnpenalized

    def _callback_robot_control_state(self, msg):
        self.blackboard.robot_control_state = msg.state

    def main_loop(self):
        """  """
        rate = rospy.Rate(25)

        while not rospy.is_shutdown() and not self.blackboard.shut_down_request:
            self.blackboard.current_time = rospy.Time.now()
            self.dsd.update() 
            try:
                # catch exception of moving backwards in time, when restarting simulator
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                rospy.logwarn(
                    "We moved backwards in time. I hope you just resetted the simulation. If not there is something wrong")
            except rospy.exceptions.ROSInterruptException:
                exit()

if __name__ == '__main__':
    try:
        LocalizationHandler()
    except rospy.ROSInterruptException:
        pass

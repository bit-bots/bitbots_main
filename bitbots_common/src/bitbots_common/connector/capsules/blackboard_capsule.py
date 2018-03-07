"""
BehaviourBlackboardCapsule
^^^^^^^^^^^^^^^^^^^^^^^^^^

"""
import os
import time
import math

import sys

import rosparam
import pickle as Pickle
import rospkg
import rospy

rospack = rospkg.RosPack()

from bitbots_pathfinding import network
from humanoid_league_msgs.msg import HeadMode
sys.modules['network'] = network

class BlackboardCapsule:
    def __init__(self):
        self.my_data = {}
        self.config_stop_g_align_dur = rosparam.get_param("Behaviour/Body/Fieldie/stopGoalAlignDuration")
        self.head_pub = None  # type: rospy.Publisher

    def freeze_till(self, ftime):
        self.my_data["freeze"] = ftime

    def is_frozen(self):
        return rospy.get_time() - self.my_data.get("freeze", 0) < 0

    ###############
    # ## Fieldie ##
    ###############
    def has_stopped_aligning(self):
        """
        Returns True if the robots has aborted or completed an aligning process in the last n seconds
        :return:
        """
        return rospy.get_time() - self.my_data.get("AligningLastStopped", 0) < self.config_stop_g_align_dur

    def stop_aligning(self):
        self.my_data["StartAlignTime"] = 0
        self.my_data["AligningLastStopped"] = rospy.get_time()

    def set_aligning_start_time(self):
        self.my_data["StartAlignTime"] = rospy.get_time()

    def get_aligning_start_time(self):
        return self.my_data.get("StartAlignTime", 0)

    ##############
    # ## Goalie ##
    ##############
    def set_thrown(self, args):
        if not args:
            self.data["WasThrownOnce"] = True
        self.my_data["ThrowDirection"] = args

    def get_throw_direction(self):
        return self.my_data.get("ThrowDirection", False)

    def was_thrown(self):
        return bool(self.my_data.get("ThrowDirection", False))

    def delete_was_thrown(self):
        self.my_data["ThrowDirection"] = False

    def get_arm_pos(self):
        return self.my_data.get("ArmPos", None)

    def set_arm_pos(self, args):
        self.my_data["ArmPos"] = args

    ################
    # ## Kick off ## #todo refactoren wenn kickoff refacored ist
    ################
    def unset_dont_need_ball(self):
        self.data["Behaviour.DontNeedBall"] = 0

    def set_enemy_kick_off_done(self):
        self.data["EnemyKickOffDone"] = True

    def has_performed_kickoff(self):
        return self.my_data.get("hasPerformedKickoff", False)

    def set_performed_kickoff(self):
        self.my_data["hasPerformedKickoff"] = True

    def unset_enemy_kick_off_done(self):
        self.data["EnemyKickOffDone"] = False

    def get_enemy_kick_off_done(self):
        return self.data.get("EnemyKickOffDone", False)

    ##################
    # ## Duty Flags ##
    ##################

    def set_one_time_kicker_timer(self, key):
        self.my_data["OneTimeKickerTimer"] = key

    def get_one_time_kicker_timer(self):
        return self.my_data.get("OneTimeKickerTimer", False)

    def set_one_time_kicked(self, key):
        self.my_data["OneTimeKicked"] = key

    def get_one_time_kicked(self):
        return self.my_data.get("OneTimeKicked", False)

    def set_goalie_out_of_goal(self, key):
        self.my_data["OutOfGoal"] = key

    def get_goalie_out_of_goal(self):
        return self.my_data.get("OutOfGoal", False)

    def set_duty(self, duty):
        self.my_data["Duty"] = duty
        role = "Other"
        if duty == "Goalie":
            role = "Goalie"
        if duty in ("Fieldie", "TeamPlayer"):
            role = "Supporter"
        self.my_data["Duty"] = duty
        # TODO role Duty auseinaderdividieren

    def get_duty(self):
        return self.my_data.get("Duty", None)

    #####################
    # ## Tracking Part ##
    #####################

    def set_head_duty(self, head_duty):
        head_duty_msg = HeadMode()
        if head_duty == "BALL_MODE":
            head_duty_msg.headMode = HeadMode.BALL_MODE
        elif head_duty == "GOAL_MODE":
            head_duty_msg.headMode = HeadMode.GOAL_MODE
        elif head_duty == "BALL_GOAL_TRACKING":
            head_duty_msg.headMode = HeadMode.BALL_GOAL_TRACKING
        elif head_duty == "FIELD_FEATURES":
            head_duty_msg.headMode = HeadMode.FIELD_FEATURES
        elif head_duty == "NON_FIELD_FEATURES":
            head_duty_msg.headMode = HeadMode.NON_FIELD_FEATURES
        elif head_duty == "LOOK_DOWN":
            head_duty_msg.headMode = HeadMode.LOOK_DOWN
        elif head_duty == "LOOK_FORWARD":
            head_duty_msg.headMode = HeadMode.LOOK_FORWARD
        elif head_duty == "DONT_MOVE":
            head_duty_msg.headMode = HeadMode.DONT_MOVE
        elif head_duty == "LOOK_UP":
            head_duty_msg.headMode = HeadMode.LOOK_UP
        else:
            rospy.logwarn("Unknown head duty %s" % head_duty)
            return
        self.head_pub.publish(head_duty_msg)

    ####################
    # ## Penalty Kick ##
    ####################

    def set_first_steps_done(self):
        self.my_data["Penalty.FirstStepsDone"] = True

    def get_first_steps_done(self):
        return self.my_data.get("Penalty.FirstStepsDone", False)

    def set_first_kick_done(self):
        self.my_data["Penalty.FirstKickDone"] = True

    def unset_first_kick_done(self):
        self.my_data["Penalty.FirstKickDone"] = False

    def get_first_kick_done(self):
        return self.my_data.get("Penalty.FirstKickDone", False)

    # forces all behaviour to use the strong kick
    def set_force_hard_kick(self):
        self.my_data["Penalty.ForceHardKick"] = True

    def unset_force_hard_kick(self):
        self.my_data["Penalty.ForceHardKick"] = False

    def get_force_hard_kick(self):
        return self.my_data.get("Penalty.ForceHardKick", False)

    ############################
    # ## Technical Challanges ##
    ############################

    ################
    # ## Throw In ##
    ################

    def get_throwin_turned(self):
        return self.my_data.get("ThrowIn.Turned", False)

    def set_throwin_turned(self):
        self.my_data["ThrowIn.Turned"] = True

    def get_throwin_aligned(self):
        return self.my_data.get("ThrowIn.Aligned", False)

    def set_throwin_aligned(self):
        self.my_data["ThrowIn.Aligned"] = True

    def set_obstacle_position(self, uv):
        self.my_data["ThrowIn.NearestObstacle"] = uv

    def get_obstacle_position(self):
        return self.my_data.get("ThrowIn.NearestObstacle", (0, 0))

    def set_no_head_movement_at_all(self):
        self.my_data["Head.NoMovementAtAll"] = True

    def is_no_head_movement_at_all(self):
        return self.my_data.get("Head.NoMovementAtAll", False)

    ########################
    # ## Hack Align Stuff ## #todo das heir sollte raus wenn wir kein hack align mehr tun
    ########################

    def get_complete_goal_found(self):
        return self.data.get("CompleteGoalFound", False)

    def unset_cant_find_complete_goal(self):
        self.data["CantFindCompleteGoal"] = False

    def get_cant_find_complete_goal(self):
        return self.data.get("CantFindCompleteGoal", False)

    def get_complete_goal_center(self):
        return self.data.get("CompleteGoalCenter", (0, 0))

    def get_complete_goal_distance(self):
        return math.sqrt(
            self.data.get("CompleteGoalCenter", (0, 0))[0] ** 2 + self.data.get("CompleteGoalCenter", (0, 0))[1] ** 2)

    def set_finished_align(self):
        self.my_data["FinishedAlign"] = rospy.get_time()

    def unset_finished_align(self):
        self.my_data["FinishedAlign"] = 0

    def get_finished_align(self):
        return rospy.get_time() - self.my_data.get("FinishedAlign", 0) < 10

    def get_pathfinding_net(self):
        if "pathfinding_net" not in self.my_data:
            path = os.path.join(rospack.get_path("bitbots_pathfinding"), "include/network.pickle")
            self.my_data["pathfinding_net"] = Pickle.load(open(path, "rb"))
        return self.my_data["pathfinding_net"]


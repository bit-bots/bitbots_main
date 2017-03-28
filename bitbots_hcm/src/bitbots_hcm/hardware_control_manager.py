#!/usr/bin/env python3.5
import argparse
import time
from math import asin

import math
from threading import Lock

import actionlib
import numpy
import rospy

import humanoid_league_msgs
from bitbots_common.pose.pypose import PyPose as Pose
from bitbots_common.util.pose_to_message import pose_to_traj_msg
from humanoid_league_speaker.speaker import speak
from std_msgs.msg import Bool, String

from bitbots_cm730.srv import SwitchMotorPower

from bitbots_hcm.hcm_state_machine import HcmStateMachine, STATE_CONTROLABLE, AnimationRunning, STATE_WALKING
from dynamic_reconfigure.server import Server
from humanoid_league_msgs.msg import RobotControlState, Animation
from humanoid_league_msgs.msg import Speak
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from bitbots_hcm.values import VALUES
from bitbots_hcm.cfg import hcm_paramsConfig


class Motion:
    def __init__(self, dieflag, standupflag, softoff_flag, softstart, start_test):

        # --- Class Variables ---
        # Setup
        self.startup_time = time.time()
        self.first_run = True

        # IMU
        self.accel = numpy.array([0, 0, 0])
        self.gyro = numpy.array([0, 0, 0])
        self.smooth_accel = numpy.array([0, 0, 0])
        self.smooth_gyro = numpy.array([0, 0, 0])
        self.not_much_smoothed_gyro = numpy.array([0, 0, 0])
        self.last_gyro_update_time = time.time()

        # Motor Positions
        self.robo_pose = Pose()
        self.goal_pose = Pose()
        self.walking_motor_goal = None
        self.walking_goal_lock = Lock()
        self.last_walking_update = 0
        self.head_goal_lock = Lock()

        # Animation
        self.animation_running = False  # animation request from animation server
        self.animation_request_time = 0  # time we got the animation request

        robot_type_name = rospy.get_param("robot_type_name")
        self.used_motor_cids = rospy.get_param("cm730/" + robot_type_name + "/motors")
        self.used_motor_names = Pose().get_joint_names_cids(self.used_motor_cids)

        # pre defiened messages for performance
        self.traj_msg = JointTrajectory()
        self.traj_msg.joint_names = [x.decode() for x in self.used_motor_names]
        self.traj_point = JointTrajectoryPoint()

        # --- Initialize Node ---
        log_level = rospy.DEBUG if rospy.get_param("debug_active", False) else rospy.INFO
        rospy.init_node('bitbots_hcm', log_level=log_level, anonymous=False)
        rospy.sleep(0.1)  # Otherwise messages will get lost, bc the init is not finished
        rospy.loginfo("Starting hcm")

        self.joint_goal_publisher = rospy.Publisher('motor_goals', JointTrajectory, queue_size=1)
        self.hcm_state_publisher = rospy.Publisher('robot_state', RobotControlState, queue_size=10, latch=True)
        self.speak_publisher = rospy.Publisher('speak', Speak, queue_size=10)
        VALUES.speak_publisher = self.speak_publisher

        rospy.sleep(0.1)  # important to make sure the connection to the speaker is established, for next line
        speak("Starting hcm", self.speak_publisher, priority=Speak.HIGH_PRIORITY)

        self.state_machine = HcmStateMachine(dieflag, standupflag, softoff_flag, softstart, start_test,
                                             self.hcm_state_publisher)

        rospy.Subscriber("imu", Imu, self.update_imu)
        rospy.Subscriber("walking_motor_goals", JointTrajectory, self.walking_goal_callback, queue_size=10)
        rospy.Subscriber("animation", Animation, self.animation_callback)
        rospy.Subscriber("head_motor_goals", JointTrajectory, self.head_goal_callback)
        rospy.Subscriber("record_motor_goals", JointTrajectory, self.record_goal_callback)
        rospy.Subscriber("pause", Bool, self.pause)

        self.animation_action_client = actionlib.SimpleActionClient('animation',
                                                                    humanoid_league_msgs.msg.PlayAnimationAction)
        VALUES.animation_client = self.animation_action_client

        self.dyn_reconf = Server(hcm_paramsConfig, self.reconfigure)

        self.main_loop()

    def pause(self, msg):
        """ Updates the pause state for the state machine"""
        VALUES.penalized = msg.data

    def update_imu(self, msg):
        """Gets new IMU values and computes the smoothed values of these"""
        update_time = time.time()
        self.accel = numpy.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.gyro = numpy.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        # todo check if this is needed by something else in the software
        # todo make smoothing factors reconfigurable
        # Remember smoothed gyro values
        # Used for falling detectiont, smooth_gyro is to late but peaks have to be smoothed anyway
        # increasing smoothing -->  later detection
        # decreasing smoothing --> more false positives
        self.smooth_gyro = numpy.multiply(self.smooth_gyro, 0.9) + numpy.multiply(self.gyro, 0.1)  # gyro
        self.smooth_accel = numpy.multiply(self.smooth_accel, 0.9) + numpy.multiply(self.accel, 0.1)  # accel
        self.not_much_smoothed_gyro = numpy.multiply(self.not_much_smoothed_gyro, 0.5) + numpy.multiply(self.gyro, 0.5)

        VALUES.raw_gyro = self.gyro
        VALUES.smooth_gyro = self.smooth_gyro
        VALUES.not_so_smooth_gyro = self.not_much_smoothed_gyro
        VALUES.smooth_accel = self.smooth_accel

        self.last_gyro_update_time = update_time
        VALUES.last_hardware_update = time.time()


    def reconfigure(self, config, level):
        """ Dynamic reconfigure of the fall checker values."""
        # just pass on to the StandupHandler, as all the variables are located there
        VALUES.fall_checker.update_reconfigurable_values(config, level)
        return config

    def walking_goal_callback(self, msg):
        VALUES.walking_active = True
        self.last_walking_update = time.time()
        if self.state_machine.get_current_state() == STATE_CONTROLABLE or \
                        self.state_machine.get_current_state() == STATE_WALKING: #todo change to statemachin.is_walking
            self.joint_goal_publisher.publish(msg)

    def head_goal_callback(self, msg):
        if not self.state_machine.is_penalized():
            # we can move our head
            self.joint_goal_publisher.publish(msg)

    def record_goal_callback(self, msg):
        if msg is None:
            # record tells us that its finished
            VALUES.record = False
        else:
            VALUES.record = True
            self.joint_goal_publisher.publish(msg)

    def animation_callback(self, msg):
        """ The animation server is sending us goal positions for the next keyframe"""
        VALUES.last_request = msg.header.stamp.to_sec()
        self.animation_request_time = time.time()
        if msg.first:
            self.animation_running = True
            VALUES.external_animation_finished = False
            if msg.hcm:
                # comming from ourselves
                # state machine already know that we're playing it, but we set the value to be sure
                VALUES.hcm_animation_playing = True
                VALUES.hcm_animation_finished = False
            else:
                # comming from outside
                if self.state_machine.get_current_state() != STATE_CONTROLABLE:
                    rospy.logwarn("Motion is not controllable, animation refused.")  # todo handle this now
                    # animation has to wait
                    # state machine should try to become controllable
                    VALUES.animation_requested = True
                    return
                else:
                    # we're already controllable, go to animation running
                    VALUES.external_animation_playing = True

        if msg.last:
            # todo reset motor speeds afterward to 0
            # todo reset pid values
            if msg.hcm:
                # This was an animation from the state machine
                VALUES.hcm_animation_playing = False
                VALUES.hcm_animation_finished = True
            else:
                # this is the last frame, we want to tell the state machine, that we're finished with the animations
                self.animation_running = False
                VALUES.external_animation_finished = True
                if msg.position is None:
                    # probably this was just to tell us we're finished
                    # we don't need to set another position to the motors
                    return

        # update goal pose
        # self.goal_pose.set_positions(list(msg.state.name), list(msg.state.position))
        # self.goal_pose.set_speeds(list(msg.state.name), list(msg.state.velocity))
        # self.publish_motor_goals()

        # forward positions to cm730, if some where transmitted
        if len(msg.position.points) > 0:
            self.joint_goal_publisher.publish(msg.position)

    def main_loop(self):
        """ Calls :func:`update_once` until ROS is shutting down """
        iteration = 0
        duration_avg = 0
        start = time.time()
        rate = rospy.Rate(100)

        while not rospy.is_shutdown(): #todo should directly be not statemachine.shutdown()
            finished = self.update_once()
            if finished:
                # Todo maybe do some last shutdown stuff after internal shutdown?
                return

            # Count to get the update frequency
            iteration += 1
            if iteration < 100:
                continue

            if duration_avg > 0:
                duration_avg = 0.5 * duration_avg + 0.5 * (time.time() - start)
            else:
                duration_avg = (time.time() - start)

            # rospy.logwarn("Updates/Sec %f", iteration / duration_avg)
            iteration = 0
            start = time.time()
            rate.sleep()

        # we got external shutdown, tell it to the state machine, it will handle it
        VALUES.shut_down = True
        # now wait for it finishing the shutdown procedure #todo fix this see above
        # while not self.state_machine.is_shutdown():
        #    # we still have to update everything
        #    self.update_once()
        #    rospy.sleep(0.01)

    def update_once(self):  # todo flag setzen falls werte geändert und nur dann evaluieren
        # check if we're still walking
        if time.time() - self.last_walking_update > 0.5:
            VALUES.walking_active = False

        # let statemachine run
        self.state_machine.evaluate()

        # now do corresponding actions depending on state of state machine
        if self.state_machine.is_shutdown():
            # the hcm has to shutdown, we tell main_loop to close the node
            return True

        # todo the following two ifs are normally unnecessary
        if self.state_machine.is_record():
            # we are currently in record mode
            # the motor goals are set directly in the callback method, so we don't have to do anything
            return

        if self.animation_running and rospy.get_time() - self.animation_request_time < 1:
            # we are currently running an animation
            # the motor goals are set directly in the callback method, so we don't have to do anything
            return


def calculate_robot_angles(raw):
    pitch_angle = calc_sin_angle(raw, numpy.array([0, 1, 0]))
    if raw[2] < 0 and raw[1] < 0:
        pitch_angle = - pitch_angle - 180
    elif raw[2] < 0 and raw[1] > 0:
        pitch_angle = 180 - pitch_angle

    roll_angle = calc_sin_angle(raw, numpy.array([1, 0, 0]))

    # TODO mir ist noch keiner schlaue Formel für diesen Wingkel eingefallen
    yaw_angle = 0

    return -roll_angle, -pitch_angle, yaw_angle


def calc_sin_angle(fst, sec):
    fst_norm = numpy.linalg.norm(fst)
    sec_norm = numpy.linalg.norm(sec)
    if fst_norm == 0 or sec_norm == 0:
        return 0  # TODO Rückgabewert sinvoll?
    return math.degrees(asin(numpy.dot(fst, sec) / (fst_norm * sec_norm)))


def main():
    parser = argparse.ArgumentParser(description='Start the hcm node')
    parser.add_argument('--no', dest='dieflag', action='store_false',
                        help='Supress the autmatical deactivating of the hcm after some time without updates')
    parser.add_argument('--nostandup', dest='standup', action='store_false',
                        help='Surpress automatical stand up')
    parser.add_argument('--softoff', dest='soft', action='store_true',
                        help='Only deactivate motors when robot is not moving')
    parser.add_argument('--softstart', dest='softstart', action='store_true',
                        help='Direclty start in softoff')
    parser.add_argument('--starttest', dest='starttest', action='store_true',
                        help='Ping motors on startup')
    args, unknown = parser.parse_known_args()

    hcm = Motion(dieflag=args.dieflag, standupflag=args.standup,
                    softoff_flag=args.soft, softstart=args.softstart,
                    start_test=args.starttest)


if __name__ == "__main__":
    main()

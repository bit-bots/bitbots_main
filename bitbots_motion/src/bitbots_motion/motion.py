#!/usr/bin/env python
#  -*- coding: utf8 -*-
import argparse
import time
from math import asin

import math
import rospy
from bitbots_common.pose.pypose import PyPose as Pose
from bitbots_speaker.speaker import speak
from std_msgs.msg import Bool

from bitbots_cm730.srv import SwitchMotorPower

from bitbots_motion.motion_state_machine import MotionStateMachine, STATE_CONTROLABLE, AnimationRunning
from dynamic_reconfigure.server import Server
from humanoid_league_msgs.msg import MotionState
from humanoid_league_msgs.msg import Speak
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from bitbots_common.utilCython.pydatavector import PyIntDataVector as IntDataVector
from bitbots_common.utilCython.pydatavector import PyDataVector as DataVector
from bitbots_common.utilCython.kalman import TripleKalman

from bitbots_animation.srv import AnimationFrame
from bitbots_motion.values import VALUES
from bitbots_motion.cfg import motion_paramsConfig


class Motion(object):
    def __init__(self, dieflag, standupflag, softoff_flag, softstart, start_test):

        # --- Class Variables ---
        # Setup
        self.startup_time = time.time()
        self.first_run = True

        # IMU
        self.accel = IntDataVector(0, 0, 0)
        self.gyro = IntDataVector(0, 0, 0)
        self.smooth_accel = IntDataVector(0, 0, 0)
        self.smooth_gyro = IntDataVector(0, 0, 0)
        self.not_much_smoothed_gyro = IntDataVector(0, 0, 0)
        self.gyro_kalman = TripleKalman()
        self.last_gyro_update_time = time.time()

        # Motor Positions
        self.robo_pose = Pose()
        self.goal_pose = Pose()
        self.walking_motor_goal = None
        self.head_motor_goal = None

        # Animation
        self.animation_running = False  # animation request from animation server
        self.animation_request_time = 0  # time we got the animation request

        # --- Initialize Node ---
        log_level = rospy.DEBUG if rospy.get_param("/debug_active", False) else rospy.INFO
        rospy.init_node('bitbots_motion', log_level=log_level, anonymous=False)
        rospy.sleep(0.1)  # This is important! Otherwise a lot of messages will get lost, bc the init is not finished
        rospy.loginfo("Starting motion")

        self.joint_goal_publisher = rospy.Publisher('/motion_motor_goals', JointState, queue_size=10)
        self.motion_state_publisher = rospy.Publisher('/motion_state', MotionState, queue_size=10)
        self.speak_publisher = rospy.Publisher('/speak', Speak, queue_size=10)
        VALUES.speak_publisher = self.speak_publisher

        rospy.sleep(0.1)  # important to make sure the connection to the speaker is established, for next line
        speak("Starting motion", self.speak_publisher, priority=Speak.HIGH_PRIORITY)

        self.state_machine = MotionStateMachine(dieflag, standupflag, softoff_flag, softstart, start_test,
                                                self.motion_state_publisher)

        rospy.Subscriber("/imu", Imu, self.update_imu)
        rospy.Subscriber("/current_motor_positions", JointState, self.update_current_pose)
        rospy.Subscriber("/walking_motor_goals", JointTrajectory, self.walking_goal_callback)
        rospy.Subscriber("/head_motor_goals", JointTrajectory, self.head_goal_callback)
        rospy.Subscriber("/record_motor_goals", JointTrajectory, self.record_goal_callback)
        rospy.Subscriber("/pause", Bool, self.pause)

        self.animation_keyframe_service = rospy.Service("animation_key_frame", AnimationFrame, self.keyframe_callback)
        self.dyn_reconf = Server(motion_paramsConfig, self.reconfigure)

        self.main_loop()

    def pause(self, msg):
        """ Updates the pause state for the state machine"""
        VALUES.penalized = msg

    def update_imu(self, msg):
        """Gets new IMU values and computes the smoothed values of these"""
        update_time = time.time()
        # todo check if this is not switched
        self.gyro = msg.linear_velocity
        self.accel = msg.angular_velocity

        # todo check if this is needed by something else in the software
        # todo make smoothing factors reconfigurable
        # Remember smoothed gyro values
        # Used for falling detectiont, smooth_gyro is to late but peaks have to be smoothed anyway
        # increasing smoothing -->  later detection
        # decreasing smoothing --> more false positives
        self.smooth_gyro = self.smooth_gyro * 0.9 + self.gyro * 0.1  ###gyro
        self.smooth_accel = self.smooth_accel * 0.9 + self.accel * 0.1  ###accel
        self.not_much_smoothed_gyro = self.not_much_smoothed_gyro * 0.5 + self.gyro * 0.5

        VALUES.raw_gyro = self.gyro
        VALUES.smooth_gyro = self.smooth_gyro
        VALUES.not_so_smooth_gyro = self.not_much_smoothed_gyro

        # calculate the angle of the robot based on the kalman filter
        angles = calculate_robot_angles(self.accel.get_data_vector())
        dt = time.time() - self.last_gyro_update_time
        angles = self.gyro_kalman.get_angles_pvv(angles, self.gyro - IntDataVector(512, 512, 512), dt)
        self.robo_angle = angles

        VALUES.robo_angle = self.robo_angle

        self.last_gyro_update_time = update_time

    def update_current_pose(self, msg):
        """Gets the current motor positions and updates the representing pose accordingly."""
        VALUES.last_client_update = msg.header.stamp
        set_joint_state_on_pose(msg, self.robo_pose) #todo implement

    def reconfigure(self, config, level):
        """ Dynamic reconfigure of the fall checker values."""
        # just pass on to the StandupHandler, as all the variables are located there
        VALUES.fall_checker.update_reconfigurable_values(config, level)
        return config

    def publish_motor_goals(self):
        """ Publish the goal pose as joint message"""
        # we can only handle one point and not a full trajectory
        msg = JointTrajectoryPoint()
        msg.positions = self.goal_pose.get_positions()
        msg.velocities = self.goal_pose.get_speeds()
        traj_msg = JointTrajectory()
        traj_msg.points = []
        traj_msg.points.append(msg)
        traj_msg.header.stamp = rospy.Time.now()
        self.joint_goal_publisher.publish(traj_msg)

    def walking_goal_callback(self, msg):
        self.walking_motor_goal = msg
        VALUES.walking_active = True

    def head_goal_callback(self, msg):
        self.head_motor_goal = msg

    def record_goal_callback(self, msg):
        if msg is None:
            # record tells us that its finished
            VALUES.record = False
        else:
            VALUES.record = True
            self.joint_goal_publisher.publish(msg)

    def keyframe_callback(self, req):
        """ The animation server is sending us goal positions for the next keyframe"""
        VALUES.last_request = req.header.stamp
        self.animation_request_time = time.time()
        if req.first_frame:
            self.animation_running = True
            VALUES.external_animation_finished = False
            if req.force:
                # force to run directly the animation
                # this is a bit hacky, but I don't know a better solution
                self.state_machine.set_state(AnimationRunning())
                # todo walking stop signal
                # todo check if motor power active
            else:
                if self.state_machine.get_current_state() != STATE_CONTROLABLE:
                    # animation has to wait
                    # state machine should try to become controllable
                    VALUES.animation_requested = True
                    return False
                else:
                    # we're already controllable, go to animation running
                    VALUES.external_animation_play = True

        if req.last_frame:
            # this is the last frame, we want to tell the state machine, that we're finished with the animations
            self.animation_running = False
            VALUES.external_animation_finished = True
            if req.positions is None:
                # probably this was just to tell us we're finished
                # we don't need to set another position to the motors
                return

        # sending keyframe positions to hardware
        self.joint_goal_publisher.publish(req.positions)
        return True

    def main_loop(self):
        """ Calls :func:`update_once` until ROS is shutting down """

        while not rospy.is_shutdown():
            finished = self.update_once()
            if finished:
                # Todo maybe do some last shutdown stuff after internal shutdown?
                return

        # we got external shutdown, tell it to the state machine, it will handle it
        VALUES.shut_down = True
        # now wait for it finishing the shutdown procedure
        while not self.state_machine.is_shutdown():
            # we still have to update everything
            self.update_once()


    def update_once(self):
        # check if we're still walking
        if self.walking_motor_goal is None or rospy.Time.now() - self.walking_motor_goal.header.stamp > 0.5:
            VALUES.walking_active = False

        # let statemachine run
        self.state_machine.evaluate()

        # now do corresponding actions depending on state of state machine

        if self.state_machine.is_shutdown():
            # the motion has to shutdown, we tell main_loop to close the node
            return True

        if self.state_machine.is_record():
            # we are currently in record mode
            # the motor goals are set directly in the callback method, so we don't have to do anything
            return

        if self.animation_running and rospy.Time.now() - self.animation_request_time < 1:
            # we are currently running an animation
            # the motor goals are set directly in the callback method, so we don't have to do anything
            return

        if self.state_machine.is_walking():
            # we're currently walking
            # set positions from first point of trajectory
            point = self.walking_motor_goal.points[0]
            self.goal_pose.set_positions(point.positions)
            self.goal_pose.set_speed(point.velocities)

        if not self.state_machine.is_penalized():
            # we can move our head
            if self.head_motor_goal is not None:
                point = self.head_motor_goal.points[0]
                self.goal_pose.set_positions(point.positions)
                self.goal_pose.set_speed(point.velocities)

        # if we didn't return yet, there are some goals to publish
        #todo maybe check, if goals are different from the last published ones
        self.publish_motor_goals()

def calculate_robot_angles(rawData):
    raw = DataVector(rawData.get_x(), rawData.get_y(), rawData.get_z())

    pitch_angle = calc_sin_angle(raw, DataVector(0, 1, 0))
    if raw.z() < 0 and raw.y() < 0:
        pitch_angle = - pitch_angle - 180
    elif (raw.z() < 0 and raw.y() > 0):
        pitch_angle = 180 - pitch_angle

    roll_angle = calc_sin_angle(raw, DataVector(1, 0, 0))

    # TODO mir ist noch keiner schlaue Formel für diesen Wingkel eingefallen
    yaw_angle = 0

    return -roll_angle, -pitch_angle, yaw_angle


def calc_sin_angle(fst, sec):
    if fst.norm() == 0 or sec.norm() == 0:
        return 0  # TODO Rückgabewert sinvoll?
    return math.degrees(asin(fst.dot(sec) / (fst.norm() * sec.norm())))


def main():
    parser = argparse.ArgumentParser(description='Start the motion node')
    parser.add_argument('--no', dest='dieflag', action='store_false',
                        help='Supress the autmatical deactivating of the motion after some time without updates')
    parser.add_argument('--nostandup', dest='standup', action='store_false',
                        help='Surpress automatical stand up')
    parser.add_argument('--softoff', dest='soft', action='store_true',
                        help='Only deactivate motors when robot is not moving')
    parser.add_argument('--softstart', dest='softstart', action='store_true',
                        help='Direclty start in softoff')
    parser.add_argument('--starttest', dest='starttest', action='store_true',
                        help='Ping motors on startup')
    args, unknown = parser.parse_known_args()

    motion = Motion(dieflag=args.dieflag, standupflag=args.standup,
                    softoff_flag=args.soft, softstart=args.softstart,
                    start_test=args.starttest)


if __name__ == "__main__":
    main()

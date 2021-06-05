#!/usr/bin/env python3

import os
import math
import re
import socket
import time

import rospy
import rospkg
import struct
import copy
from threading import Lock

import robocup_extension_pb2


class HumanoidLeagueTeamCommunication:
    def __init__(self):
        rospack = rospkg.RosPack()
        self._package_path = rospack.get_path("humanoid_league_team_communication")

        rospy.init_node("humanoid_league_team_communication")
        rospy.loginfo("Initializing humanoid_league_team_communication...", logger_name="team_comm")

        self.create_publishers()
        self.create_subscribers()

        # we will try multiple times till we manage to get a connection
        self.socket = None
        while not rospy.is_shutdown() and self.socket is None:
            self.socket = self.get_connection(addr)
            time.sleep(1) #dont use ros time since it is maybe not available

        self.first_run = True
        self.published_camera_info = False

        self.joint_command_mutex = Lock()

        self.run()

    def receive_msg(self):

        # self.socket.recvfrom(GameState.sizeof())

        msg_size = self.socket.recv(4)
        msg_size = struct.unpack(">L", msg_size)[0]

        data = bytearray()
        while len(data) < msg_size:
            packet = self.socket.recv(msg_size - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data

    def run(self):
        while not rospy.is_shutdown():
            # Parse sensor
            msg = self.receive_msg()
            self.handle_sensor_measurements_msg(msg)

            sensor_time_steps = None
            if self.first_run:
                sensor_time_steps = self.get_sensor_time_steps(active=True)
            self.send_actuator_requests(sensor_time_steps)
            self.first_run = False
        self.close_connection()

    def create_publishers(self):
        self.pub_clock = rospy.Publisher(rospy.get_param('~clock_topic'), Clock, queue_size=1)
        self.pub_server_time_clock = rospy.Publisher(rospy.get_param('~server_time_clock_topic'), Clock, queue_size=1)
        self.pub_camera = rospy.Publisher(rospy.get_param('~camera_topic'), Image, queue_size=1)
        self.pub_camera_info = rospy.Publisher(rospy.get_param('~camera_info_topic'), CameraInfo, queue_size=1, latch=True)
        self.pub_imu = rospy.Publisher(rospy.get_param('~imu_topic'), Imu, queue_size=1)
        self.pub_head_imu = rospy.Publisher(rospy.get_param('~imu_head_topic'), Imu, queue_size=1)
        self.pub_pressure_left = rospy.Publisher(rospy.get_param('~foot_pressure_left_topic'), FootPressure, queue_size=1)
        self.pub_pressure_right = rospy.Publisher(rospy.get_param('~foot_pressure_right_topic'), FootPressure, queue_size=1)
        self.pub_cop_l = rospy.Publisher(rospy.get_param('~cop_left_topic'), PointStamped, queue_size=1)
        self.pub_cop_r_ = rospy.Publisher(rospy.get_param('~cop_right_topic'), PointStamped, queue_size=1)
        self.pub_joint_states = rospy.Publisher(rospy.get_param('~joint_states_topic'), JointState, queue_size=1)

    def create_subscribers(self):
        self.sub_joint_command = rospy.Subscriber(rospy.get_param('~joint_command_topic'), JointCommand, self.joint_command_cb, queue_size=1)

    def joint_command_cb(self, msg):
        with self.joint_command_mutex:
            self.joint_command = msg

    def get_connection(self, addr):
        host, port = addr.split(':')
        port = int(port)
        rospy.loginfo(f"Connecting to '{addr}'", logger_name="rc_api")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect((host, port))
            response = sock.recv(8).decode('utf8')
        except ConnectionRefusedError:
            rospy.logerr(f"Connection refused by '{addr}'", logger_name="rc_api")
            return None
        if response == "Welcome\0":
            rospy.loginfo(f"Successfully connected to '{addr}'", logger_name="rc_api")
            return sock
        elif response == "Refused\0":
            rospy.logerr(f"Connection refused by '{addr}'", logger_name="rc_api")
            return None
        else:
            rospy.logerr(f"Could not connect to '{addr}'\nGot response '{response}'", logger_name="rc_api")
            return None

    def close_connection(self):
        self.socket.close()

    def handle_sensor_measurements_msg(self, msg):
        s_m = messages_pb2.SensorMeasurements()
        s_m.ParseFromString(msg)

        self.handle_time(s_m.time)
        self.handle_real_time(s_m.real_time)
        self.handle_messages(s_m.messages)
        self.handle_imu_data(s_m.accelerometers, s_m.gyros)
        self.handle_bumper_measurements(s_m.bumpers)
        self.handle_camera_measurements(s_m.cameras)
        self.handle_force_measurements(s_m.forces)
        self.handle_force3D_measurements(s_m.force3ds)
        self.handle_force6D_measurements(s_m.force6ds)
        self.handle_position_sensor_measurements(s_m.position_sensors)


if __name__ == '__main__':
    HumanoidLeagueTeamCommunication()

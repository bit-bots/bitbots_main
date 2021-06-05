#!/usr/bin/env python3

import socket

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

        self.config = rospy.get_param("~")

        self.host = self.config['host']
        self.port = self.config['port']

        self.create_publishers()
        self.create_subscribers()

        # we will try multiple times till we manage to get a connection
        self.socket = None
        while not rospy.is_shutdown() and self.socket is None:
            self.socket = self.get_connection()
            rospy.time.sleep(1)

        self.run()

    def create_publishers(self):
        pass

    def create_subscribers(self):
        pass

    def get_connection(self):
        rospy.loginfo(f"Connecting to '{self.host}:{self.port}'", logger_name="team_comm")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect((self.host, self.port))

    def close_connection(self):
        self.socket.close()
        rospy.loginfo("Connection closed.", logger_name="team_comm")

    def receive_msg(self):
        pass
        # self.socket.recvfrom(GameState.sizeof())

        # msg_size = self.socket.recv(4)
        # msg_size = struct.unpack(">L", msg_size)[0]

        # data = bytearray()
        # while len(data) < msg_size:
        #     packet = self.socket.recv(msg_size - len(data))
        #     if not packet:
        #         return None
        #     data.extend(packet)
        # return data

    def run(self):
        while not rospy.is_shutdown():
            # Parse sensor
            msg = self.receive_msg()
            self.handle_message(msg)

        self.close_connection()

    def handle_message(self, msg):
        message = robocup_extension_pb2.Message()
        message.ParseFromString(msg)
        print(message)


if __name__ == '__main__':
    HumanoidLeagueTeamCommunication()

#!/usr/bin/env python3

import socket

import rospy
import rospkg
import struct
import copy
from threading import Lock

import transforms3d.euler
from geometry_msgs.msg import PoseWithCovariance, Twist, Pose
from humanoid_league_msgs.msg import GameState, PoseWithCertaintyArray

import robocup_extension_pb2
from google.protobuf.timestamp_pb2 import Timestamp


class HumanoidLeagueTeamCommunication:
    def __init__(self):
        rospack = rospkg.RosPack()
        self._package_path = rospack.get_path("humanoid_league_team_communication")

        rospy.init_node("humanoid_league_team_communication")
        rospy.loginfo("Initializing humanoid_league_team_communication...", logger_name="team_comm")

        self.config = rospy.get_param("~")

        self.host = self.config['host']
        self.port = self.config['port']

        self.player_id = rospy.get_param('bot_id')
        self.team_id = rospy.get_param('team_id')

        self.create_publishers()
        self.create_subscribers()

        self.gamestate = None  # type: GameState
        self.pose = None  # type: PoseWithCovariance
        self.cmd_vel = None  # type: Twist
        self.ball = None  # type: PoseWithCertainty

        # we will try multiple times till we manage to get a connection
        self.socket = None
        while not rospy.is_shutdown() and self.socket is None:
            self.socket = self.get_connection()
            rospy.sleep(1)

        self.run()

    def create_publishers(self):
        pass

    def create_subscribers(self):
        rospy.Subscriber('gamestate', GameState, self.gamestate_cb, queue_size=1)
        rospy.Subscriber('pose_with_covariance', PoseWithCovariance, self.pose_cb, queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb, queue_size=1)
        rospy.Subscriber('balls_relative', PoseWithCertaintyArray, self.ball_cb, queue_size=1)

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

    def gamestate_cb(self, msg):
        self.gamestate = msg

    def pose_cb(self, msg):
        self.pose = msg

    def cmd_vel_cb(self, msg):
        self.cmd_vel = msg

    def ball_cb(self, msg):
        self.ball = msg

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

    def send_message(self):
        message = robocup_extension_pb2.Message()
        message.timestamp = Timestamp()
        now = rospy.Time.now()
        message.timestamp.set_seconds(now.to_sec())
        message.timestamp.set_nanos(now.to_nsec())

        if self.gamestate.penalized:
            message.state = robocup_extension_pb2.State.PENALISED
        else:
            message.state = robocup_extension_pb2.State.UNPENALISED

        current_pose = robocup_extension_pb2.Robot()
        current_pose.player_id = self.player_id
        current_pose.position.x = self.pose.pose.position.x
        current_pose.position.y = self.pose.pose.position.y
        q = self.pose.pose.orientation
        # z is theta
        current_pose.position.z = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])[2]
        current_pose.team = self.team_id
        message.current_pose = current_pose

        message.walk_command.x = self.cmd_vel.linear.x
        message.walk_command.y = self.cmd_vel.linear.y
        message.walk_command.z = self.cmd_vel.angular.z

        # message.target_pose is currently not used
        # message.kick_target is currently not used

        ball = robocup_extension_pb2.Ball()


if __name__ == '__main__':
    HumanoidLeagueTeamCommunication()

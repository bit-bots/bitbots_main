#!/usr/bin/env python3

import socket
from typing import Optional

import rospy
import rospkg
import struct
import copy
from threading import Lock

import tf2_ros
import transforms3d.euler
from geometry_msgs.msg import PoseWithCovariance, Twist
from humanoid_league_msgs.msg import GameState, PoseWithCertaintyArray, TeamData, ObstacleRelativeArray, ObstacleRelative
from tf2_geometry_msgs import PointStamped

import robocup_extension_pb2
from google.protobuf.timestamp_pb2 import Timestamp


# TODO: Handle lifetime from config
# TODO: Handle belief_threshold from config

class HumanoidLeagueTeamCommunication:
    def __init__(self):
        rospack = rospkg.RosPack()
        self._package_path = rospack.get_path("humanoid_league_team_communication")

        rospy.init_node("humanoid_league_team_communication")
        rospy.loginfo("Initializing humanoid_league_team_communication...", logger_name="team_comm")

        self.player_id = rospy.get_param('bot_id')
        self.team_id = rospy.get_param('team_id')

        self.config = rospy.get_param("~")

        self.target_host = self.config['target_host']
        if self.target_host == '127.0.0.1':
            # local mode, bind to port depending on bot id
            self.target_ports = self.config['local_target_ports']
            self.receive_port = self.target_ports[self.player_id - 1]
        else:
            self.target_ports = [self.config['target_port']]
            self.receive_port = self.config['receive_port']

        self.create_publishers()
        self.create_subscribers()

        self.gamestate = None  # type: GameState
        self.pose = None  # type: PoseWithCovariance
        self.cmd_vel = None  # type: Twist
        self.ball = None  # type: Optional[PointStamped]
        self.ball_confidence = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # we will try multiple times till we manage to get a connection
        self.socket = None
        while not rospy.is_shutdown() and self.socket is None:
            self.socket = self.get_connection()
            rospy.sleep(1)

        rospy.Timer(rospy.Duration.from_sec(1 / self.config['rate']), self.send_message)
        self.receive_forever()

    def create_publishers(self):
        self.pub_team_data = rospy.Publisher(self.config['team_data'], TeamData, queue_size=1)

    def create_subscribers(self):
        # TODO: Use rostopics from config and check all in config
        rospy.Subscriber('gamestate', GameState, self.gamestate_cb, queue_size=1)
        rospy.Subscriber('pose_with_covariance', PoseWithCovariance, self.pose_cb, queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb, queue_size=1)
        rospy.Subscriber('balls_relative', PoseWithCertaintyArray, self.ball_cb, queue_size=1)

    def get_connection(self):
        rospy.loginfo(f"Binding to port {self.receive_port}", logger_name="team_comm")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.bind(('0.0.0.0', self.receive_port))
        return sock

    def close_connection(self):
        if self.socket:
            self.socket.close()
            rospy.loginfo("Connection closed.", logger_name="team_comm")

    def receive_msg(self):
        return self.socket.recv(1024)

    def gamestate_cb(self, msg):
        self.gamestate = msg

    def pose_cb(self, msg):
        self.pose = msg

    def cmd_vel_cb(self, msg):
        self.cmd_vel = msg

    def ball_cb(self, msg: PoseWithCertaintyArray):
        if msg.poses:
            # Sort by confidence
            balls = sorted(msg.poses, reverse=True, key=lambda ball: ball.confidence)
            # Highest confidence
            ball = balls[0]

            if ball.confidence == 0:
                self.ball = None
            else:
                # Transform to map
                ball_point = PointStamped(msg.header, ball.pose.pose.position)
                try:
                    self.ball = self.tf_buffer.transform(ball_point, "map", timeout=rospy.Duration.from_sec(0.3))
                    self.ball_confidence = ball.confidence
                except tf2_ros.TransformException:
                    self.ball = None

    def __del__(self):
        self.close_connection()

    def receive_forever(self):
        while not rospy.is_shutdown():
            try:
                msg = self.receive_msg()
            except (struct.error, socket.timeout):
                continue

            if msg:  # Not handle empty messages or None
                self.handle_message(msg)

    def handle_message(self, msg):
        def set_covariance(fmat3, ros_covariance):
            # ROS covariance is row-major 36 x float, protobuf covariance is column-major 9 x float [x, y, θ]
            ros_covariance[0] = fmat3.x.x
            ros_covariance[1] = fmat3.y.x
            ros_covariance[5] = fmat3.z.x
            ros_covariance[6] = fmat3.x.y
            ros_covariance[7] = fmat3.y.y
            ros_covariance[11] = fmat3.z.y
            ros_covariance[30] = fmat3.x.z
            ros_covariance[31] = fmat3.y.z
            ros_covariance[35] = fmat3.z.z

        def set_pose(robot, pose):
            pose.pose.position.x = robot.position.x
            pose.pose.position.y = robot.position.y

            quat = transforms3d.euler.euler2quat(0.0, 0.0, robot.position.z)  #wxyz -> ros: xyzw
            pose.pose.orientation.x = quat[1]
            pose.pose.orientation.y = quat[2]
            pose.pose.orientation.z = quat[3]
            pose.pose.orientation.w = quat[0]

            if pose.covariance:
                set_covariance(robot.covariance, pose.covariance)

        message = robocup_extension_pb2.Message()
        message.ParseFromString(msg)

        team_data = TeamData()

        player_id = message.current_pose.player_id
        player_team = message.current_pose.team

        # Handle timestamp
        ##################
        team_data.header.stamp.secs = message.timestamp.seconds
        team_data.header.stamp.nsecs = message.timestamp.nanos
        # TODO: FRAME_ID

        # Handle state
        ##############
        team_data.state = message.state

        # Handle pose of current player
        ###############################
        set_pose(message.current_pose, team_data.robot_position.pose)

        if hasattr(message, "position_confidence"):
            team_data.robot_position.confidence = message.position_confidence

        # Handle ball
        #############
        team_data.ball_relative.pose.pose.position.x = message.ball.position.x
        team_data.ball_relative.pose.pose.position.y = message.ball.position.y
        team_data.ball_relative.pose.pose.position.z = message.ball.position.z

        # TODO: Ball velocity

        if message.ball.covariance:
            set_covariance(message.ball.covariance, team_data.ball_relative.pose.covariance)

        # Handle obstacles
        ##################
        obstacle_relative_array = ObstacleRelativeArray()
        # TODO: Header

        obstacles = ()
        for index, robot in enumerate(message.others):
            obstacle = ObstacleRelative()

            # Obstacle type
            team_to_obstacle_type = {
                robocup_extension_pb2.Team.UNKOWN_TEAM: ObstacleRelative.ROBOT_UNDEFINED,
                robocup_extension_pb2.Team.BLUE: ObstacleRelative.ROBOT_CYAN,
                robocup_extension_pb2.Team.RED: ObstacleRelative.ROBOT_MAGENTA,
            }
            obstacle.type = team_to_obstacle_type[robot.team]

            obstacle.playerNumber = robot.player_id

            set_pose(robot, obstacle.pose.pose)
            if hasattr(message, "obstacle_confidence") and index < len(message.obstacle_confidence):
                obstacle.pose.confidence = message.obstacle_confidence[index]

            # width
            # heigth

            obstacles.append(obstacle)
        team_data.obstacles = obstacles

        # Handle time to position at ball
        #################################
        if hasattr(message, "time_to_ball"):
            team_data.time_to_position_at_ball = message.time_to_ball

        # Handle strategy
        #################
        if hasattr(message, "role"):
            team_data.strategy.role = message.role
        if hasattr(message, "action"):
            team_data.strategy.action = message.action
        if hasattr(message, "offensive_side"):
            team_data.strategy.offensive_side = message.offensive_side

        self.pub_team_data.publish(team_data)

    def send_message(self, event):
        message = robocup_extension_pb2.Message()
        now = rospy.Time.now()
        message.timestamp.seconds = now.secs
        message.timestamp.nanos = now.nsecs

        if self.gamestate:
            if self.gamestate.penalized:
                message.state = robocup_extension_pb2.State.PENALISED
            else:
                message.state = robocup_extension_pb2.State.UNPENALISED
        else:
            message.state = robocup_extension_pb2.State.UNKNOWN_STATE

        message.current_pose.player_id = self.player_id

        if self.pose:
            message.current_pose.position.x = self.pose.pose.position.x
            message.current_pose.position.y = self.pose.pose.position.y
            q = self.pose.pose.orientation
            # z is theta
            message.current_pose.position.z = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])[2]
            message.current_pose.team = self.team_id
            message.position_confidence = 1
        else:
            message.position_confidence = 0

        if self.cmd_vel:
            message.walk_command.x = self.cmd_vel.linear.x
            message.walk_command.y = self.cmd_vel.linear.y
            message.walk_command.z = self.cmd_vel.angular.z

        # message.target_pose is currently not used
        # message.kick_target is currently not used

        # TODO add a timeout to the ball?
        if self.ball is not None:
            message.ball.position.x = self.ball.point.x
            message.ball.position.y = self.ball.point.y
            message.ball.position.z = self.ball.point.z
            # ball.velocity is currently not set, wait for ball filter
            # ball.covariance is currently not set, wait for ball filter
            message.ball_confidence = self.ball_confidence
        else:
            message.ball_confidence = 0

        # message.others should be set from robot detections

        # message.max_walking_speed is currently not set
        # how should message.time_to_ball be calculated?

        # We ask every time for the role because it might change during the game
        role = rospy.get_param('role', None)
        if role == 'goalie':
            message.role = robocup_extension_pb2.Role.ROLE_GOALIE
        elif role == 'defense':
            message.role = robocup_extension_pb2.Role.ROLE_DEFENDER
        elif role == 'offense':
            message.role = robocup_extension_pb2.Role.ROLE_STRIKER
        else:
            message.role = robocup_extension_pb2.Role.ROLE_OTHER

        # where do we get message.action from?
        # do we even have an offensive side for message.offensive_side?

        # message.obstacle_confidence should probably be renamed to robot_confidences because we do not
        # publish general obstacles, only the other robots.

        msg = message.SerializeToString()
        for port in self.target_ports:
            rospy.logdebug(f'sending to {port} on {self.target_host}')
            self.socket.sendto(msg, (self.target_host, port))


if __name__ == '__main__':
    HumanoidLeagueTeamCommunication()

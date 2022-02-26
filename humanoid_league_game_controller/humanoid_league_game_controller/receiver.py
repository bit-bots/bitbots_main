#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import unicode_literals, print_function

"""
This code provides a gamecontroller client for the RoboCup Humanoid League.

.. moduleauthor:: Nils Rokita <0rokita@informatik.uni-hamburg.de>
.. moduleauthor:: Robert Kessler <8kessler@informatik.uni-hamburg.de>

"""


import socket
import time
import rclpy
from rclpy import logging
from rclpy.node import Node

from construct import Container, ConstError

from humanoid_league_msgs.msg import GameState as GameStateMsg
from std_msgs.msg import Bool
from humanoid_league_game_controller.gamestate import GameState, ReturnData, GAME_CONTROLLER_RESPONSE_VERSION

logger = logging.get_logger('humanoid_league_game_controller')

class GameStateReceiver(Node):
    """ This class puts up a simple UDP Server which receives the
    *addr* parameter to listen to the packages from the game_controller.

    If it receives a package it will be interpreted with the construct data
    structure and the :func:`on_new_gamestate` will be called with the content.

    After this we send a package back to the GC """

    def __init__(self):
        super().__init__('game_controller', automatically_declare_parameters_from_overrides=True)

        # Information that is used when sending the answer to the game controller
        # TODO: Check if parameters can be renamed -> where do they come from
        self.team_number = self.get_parameter('team_id').value
        self.player_number = self.get_parameter('bot_id').value
        logger.info('We are playing as player {} in team {}'.format(self.player_number, self.team_number))

        self.state_publisher = self.create_publisher(GameStateMsg, 'gamestate', 1)

        self.man_penalize = False
        self.game_controller_lost_time = 20
        self.game_controller_connected_publisher = self.create_publisher(Bool, 'game_controller_connected', 1)

        # The address listening on and the port for sending back the robots meta data
        listen_host = self.get_parameter('listen_host').value
        listen_port = self.get_parameter('listen_port').value
        self.addr = (listen_host, listen_port)
        self.answer_port = self.get_parameter('answer_port').value

        # The state and time we received last form the GC
        self.state = None
        self.time = time.time()

        # The socket and whether it is still running
        self.socket = None
        self.running = True

        self._open_socket()

    def _open_socket(self):
        """ Creates the socket """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.addr)
        self.socket.settimeout(2)
        self.socket2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def receive_forever(self):
        """ Waits in a loop that is terminated by setting self.running = False """
        while True:
            try:
                self.receive_once()
            except IOError as e:
                logger.warn("Error while sending keepalive: " + str(e))

    def receive_once(self):
        """ Receives a package and interprets it.
            Calls :func:`on_new_gamestate`
            Sends an answer to the GC """
        try:
            data, peer = self.socket.recvfrom(GameState.sizeof())

            # Throws a ConstError if it doesn't work
            parsed_state = GameState.parse(data)

            # Assign the new package after it parsed successful to the state
            self.state = parsed_state
            self.time = time.time()

            # Publish that game controller received message
            msg = Bool()
            msg.data = True
            self.game_controller_connected_publisher.publish(msg)

            # Call the handler for the package
            self.on_new_gamestate(self.state)

            # Answer the GameController
            self.answer_to_gamecontroller(peer)

        except AssertionError as ae:
            logger.error(ae)
        except socket.timeout:
            logger.info("No GameController message received (socket timeout)", throttle_duration_sec=5)
        except ConstError:
            logger.warn("Parse Error: Probably using an old protocol!")
        finally:
            if self.get_time_since_last_package() > self.game_controller_lost_time:
                self.time += 5  # Resend message every five seconds
                logger.info("No GameController message received, allowing robot to move", throttle_duration_sec=5)
                msg = GameStateMsg()
                msg.game_state = 3 # PLAYING
                self.state_publisher.publish(msg)
                msg2 = Bool()
                msg2.data = False
                self.game_controller_connected_publisher.publish(msg2)

    def answer_to_gamecontroller(self, peer):
        """ Sends a life sign to the game controller """
        return_message = 0 if self.man_penalize else 2

        data = Container(
            header=b"RGrt",
            version=GAME_CONTROLLER_RESPONSE_VERSION,
            team=self.team_number,
            player=self.player_number,
            message=return_message)
        try:
            destination = peer[0], self.answer_port
            logger.debug('Sending answer to {} port {}'.format(destination[0], destination[1]))
            self.socket.sendto(ReturnData.build(data), destination)
        except Exception as e:
            logger.error("Network Error: %s" % str(e))

    def on_new_gamestate(self, state):
        """ Is called with the new game state after receiving a package.
            The information is processed and published as a standard message to a ROS topic.
            :param state: Game State
        """

        is_own_team = lambda number: number == self.team_number
        own_team = self.select_team_by(is_own_team, state.teams)

        is_not_own_team = lambda number: number != self.team_number
        rival_team = self.select_team_by(is_not_own_team, state.teams)

        if not own_team or not rival_team:
            logger.error('Team {} not playing, only {} and {}'.format(self.team_number,
                                                                      state.teams[0].team_number,
                                                                      state.teams[1].team_number))
            return

        try:
            me = own_team.players[self.player_number - 1]
        except IndexError:
            logger.error('Robot {} not playing'.format(self.player_number))
            return

        msg = GameStateMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.game_state = state.game_state.intvalue
        msg.secondary_state = state.secondary_state.intvalue
        msg.secondary_state_mode = state.secondary_state_info[1]
        msg.first_half = state.first_half
        msg.own_score = own_team.score
        msg.rival_score = rival_team.score
        msg.seconds_remaining = state.seconds_remaining
        msg.secondary_seconds_remaining = state.secondary_seconds_remaining
        msg.has_kick_off = state.kick_of_team == self.team_number
        msg.penalized = me.penalty != 0
        msg.seconds_till_unpenalized = me.secs_till_unpenalized
        msg.secondary_state_team = state.secondary_state_info[0]
        msg.secondary_state_mode = state.secondary_state_info[1]
        msg.team_color = own_team.team_color.intvalue
        msg.drop_in_team = state.drop_in_team
        msg.drop_in_time = state.drop_in_time
        msg.penalty_shot = own_team.penalty_shot
        msg.single_shots = own_team.single_shots
        msg.coach_message = own_team.coach_message
        penalties = []
        red_cards = []
        for i in range(6):
            penalties.append(own_team.players[i].penalty != 0)
            red_cards.append(own_team.players[i].number_of_red_cards != 0)
        msg.team_mates_with_penalty = penalties
        msg.team_mates_with_red_card = red_cards
        self.state_publisher.publish(msg)

    def get_last_state(self):
        return self.state, self.time

    def get_time_since_last_package(self):
        return time.time() - self.time

    def stop(self):
        self.running = False

    def set_manual_penalty(self, flag):
        self.man_penalize = flag

    def select_team_by(self, predicate, teams):
        selected = [team for team in teams if predicate(team.team_number)]
        return next(iter(selected), None)

def main(args=None):
    rclpy.init(args=args)
    receiver = GameStateReceiver()

    try:
        receiver.receive_forever()
    except KeyboardInterrupt:
        receiver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

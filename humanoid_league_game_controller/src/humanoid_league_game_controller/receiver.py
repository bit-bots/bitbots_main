#!/usr/bin/env python
#-*- coding:utf-8 -*-

from __future__ import unicode_literals, print_function

"""
This module shows how the GameController Communication protocol can be used
in python and also allows to be changed such that every team using python to
interface with the GC can utilize the new protocol.

.. moduleauthor:: Nils Rokita <0rokita@informatik.uni-hamburg.de>
.. moduleauthor:: Robert Kessler <8kessler@informatik.uni-hamburg.de>

"""


import socket
import time
import rospy

from construct import Container, ConstError

from humanoid_league_msgs.msg import GameState as GameStateMsg
from gamestate import GameState, ReturnData, GAME_CONTROLLER_RESPONSE_VERSION


class GameStateReceiver(object):
    """ This class puts up a simple UDP Server which receives the
    *addr* parameter to listen to the packages from the game_controller.

    If it receives a package it will be interpreted with the construct data
    structure and the :func:`on_new_gamestate` will be called with the content.

    After this we send a package back to the GC """

    def __init__(self):
        rospy.init_node('game_controller')

        # Information that is used when sending the answer to the game controller
        self.team = rospy.get_param('/team_id')
        self.player = rospy.get_param('/bot_id')
        self.man_penalize = False

        # The address listening on and the port for sending back the robots meta data
        self.addr = (rospy.get_param('/game_controller/listen_host'), rospy.get_param('/game_controller/listen_port'))
        self.answer_port = rospy.get_param('/game_controller/answer_port')

        # The state and time we received last form the GC
        self.state = None
        self.time = None

        # The socket and whether it is still running
        self.socket = None
        self.running = True

        self._open_socket()

    def _open_socket(self):
        """ Creates the socket """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.addr)
        self.socket.settimeout(0.5)
        self.socket2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def receive_forever(self):
        """ Waits in a loop that is terminated by setting self.running = False """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            try:
                self.receive_once()
            except IOError as e:
                rospy.logwarn("Error while sending keepalive: " + str(e))
            r.sleep()

    def receive_once(self):
        """ Receives a package and interprets it.
            Calls :func:`on_new_gamestate`
            Sends an answer to the GC """
        try:
            data, peer = self.socket.recvfrom(GameState.sizeof())

            print(len(data))
            # Throws a ConstError if it doesn't work
            parsed_state = GameState.parse(data)

            # Assign the new package after it parsed successful to the state
            self.state = parsed_state
            self.time = time.time()

            # Call the handler for the package
            self.on_new_gamestate(self.state)

            # Answer the GameController
            self.answer_to_gamecontroller(peer)

        except AssertionError as ae:
            rospy.logerr(ae.message)
        except socket.timeout:
            rospy.logwarn("Socket timeout")
        except ConstError as e: 
            rospy.logwarn("Parse Error: Probably using an old protocol!")
        except Exception as e:
            rospy.logerr(e)
            pass

    def answer_to_gamecontroller(self, peer):
        """ Sends a life sign to the game controller """
        return_message = 0 if self.man_penalize else 2

        data = Container(
            header=b"RGrt",
            version=GAME_CONTROLLER_RESPONSE_VERSION,
            team=self.team,
            player=self.player,
            message=return_message)
        try:
            destination = peer[0], self.answer_port
            print('Sending to', destination)
            self.socket.sendto(ReturnData.build(data), destination)
        except Exception as e:
            rospy.logerr("Network Error: %s" % str(e))

    def on_new_gamestate(self, state):
        """ Is called with the new game state after receiving a package
            Needs to be implemented or set
            :param state: Game State
        """
        msg = GameStateMsg()
        msg.header.stamp = rospy.Time.now()
        msg.gameState = state.game_state.intvalue
        msg.secondaryState = state.secondary_state.intvalue
        msg.secondaryStateTeam # TODO
        msg.firstHalf = bool(state.first_half)
        msg.ownScore # TODO
        msg.rivalScore # TODO
        msg.secondsRemaining = state.seconds_remaining
        msg.secondary_seconds_remaining = state.secondary_seconds_remaining
        msg.hasKickOff # TODO
        msg.penalized # TODO
        msg.secondsTillUnpenalized # TODO
        msg.allowedToMove # TODO
        msg.teamColor # TODO
        msg.dropInTeam = bool(state.drop_in_team)
        msg.dropInTime = state.drop_in_time
        msg.penaltyShot # TODO
        msg.singleShots # TODO
        msg.coach_message # TODO
        print(msg)

    def get_last_state(self):
        return self.state, self.time

    def get_time_since_last_package(self):
        return time.time() - self.time

    def stop(self):
        self.running = False

    def set_manual_penalty(self, flag):
        self.man_penalize = flag

if __name__ == '__main__':
    rec = GameStateReceiver()
    rec.receive_forever()


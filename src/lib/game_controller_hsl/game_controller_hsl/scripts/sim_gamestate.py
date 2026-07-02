#!/usr/bin/env python3

# This script was based on the teleop_twist_keyboard package
# original code can be found at https://github.com/ros-teleop/teleop_twist_keyboard
# The script provides a simple mechanism to test robot behavior in different game states,
# when no game controller is running

import select
import sys
import termios
import tty

import rclpy
from game_controller_hsl.utils import get_parameters_from_other_node
from game_controller_hsl_interfaces.msg import GameState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class SimGameState(Node):
    DEFAULT_PLAYERS_PER_TEAM = 4
    STATE_KEYS = {
        "0": ("STATE_INITIAL", GameState.STATE_INITIAL),
        "1": ("STATE_READY", GameState.STATE_READY),
        "2": ("STATE_SET", GameState.STATE_SET),
        "3": ("STATE_PLAYING", GameState.STATE_PLAYING),
        "4": ("STATE_FINISHED", GameState.STATE_FINISHED),
    }
    COMPETITION_TYPE_KEYS = {
        "5": ("COMPETITION_TYPE_SMALL", GameState.COMPETITION_TYPE_SMALL),
        "6": ("COMPETITION_TYPE_MIDDLE", GameState.COMPETITION_TYPE_MIDDLE),
        "7": ("COMPETITION_TYPE_LARGE", GameState.COMPETITION_TYPE_LARGE),
    }
    GAME_PHASE_KEYS = {
        "a": ("GAME_PHASE_NORMAL", GameState.GAME_PHASE_NORMAL),
        "b": ("GAME_PHASE_PENALTY_SHOOT_OUT", GameState.GAME_PHASE_PENALTY_SHOOT_OUT),
        "c": ("GAME_PHASE_EXTRA_TIME", GameState.GAME_PHASE_EXTRA_TIME),
        "d": ("GAME_PHASE_TIMEOUT", GameState.GAME_PHASE_TIMEOUT),
    }
    SET_PLAY_KEYS = {
        "e": ("SET_PLAY_NONE", GameState.SET_PLAY_NONE),
        "f": ("SET_PLAY_DIRECT_FREE_KICK", GameState.SET_PLAY_DIRECT_FREE_KICK),
        "g": ("SET_PLAY_INDIRECT_FREE_KICK", GameState.SET_PLAY_INDIRECT_FREE_KICK),
        "h": ("SET_PLAY_PENALTY_KICK", GameState.SET_PLAY_PENALTY_KICK),
        "i": ("SET_PLAY_THROW_IN", GameState.SET_PLAY_THROW_IN),
        "j": ("SET_PLAY_GOAL_KICK", GameState.SET_PLAY_GOAL_KICK),
        "k": ("SET_PLAY_CORNER_KICK", GameState.SET_PLAY_CORNER_KICK),
    }

    def __init__(self):
        super().__init__("sim_gamestate")
        self.logger = self.get_logger()

        params = self.load_parameters()
        self.team_id = int(params["team_id"])
        self.player_number = int(params["bot_id"])
        self.has_kick = True

        self.settings = termios.tcgetattr(sys.stdin)

        self.publisher = self.create_publisher(
            GameState,
            "gamestate",
            QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1),
        )

    def load_parameters(self) -> dict[str, int | str]:
        """
        Tries fetching team_id and bot_id from parameter blackboard.
        If the blackboard is unavailable, fall back to manual/default values.
        """
        try:
            self.logger.info("Trying to fetch team_id and bot_id from parameter blackboard")
            params = get_parameters_from_other_node(
                self,
                "parameter_blackboard",
                ["team_id", "bot_id"],
                service_timeout_sec=5.0,
            )
        except TimeoutError:
            self.logger.warn("Parameter blackboard not available")
            params = {}

        team_id = params.get("team_id")
        bot_id = params.get("bot_id")

        if team_id is None:
            self.logger.warn("No team_id found in parameter blackboard")
            params["team_id"]= int(input("Please enter team id: "))

        if bot_id is None:
            self.logger.warn("No bot_id found in parameter blackboard, assuming player 1")
            params["bot_id"] = 1

        return params

    def create_game_state_msg(self):
        game_state_msg = GameState()
        game_state_msg.header.stamp = self.get_clock().now().to_msg()
        game_state_msg.players_per_team = self.DEFAULT_PLAYERS_PER_TEAM
        game_state_msg.kicking_team = self.team_id

        self.sync_team_mates(game_state_msg)
        return game_state_msg

    def apply_key(self, game_state_msg, key):
        if key == "\x03":
            return False
        elif key in self.STATE_KEYS:
            game_state_msg.main_state = self.STATE_KEYS[key][1]
        elif key in self.COMPETITION_TYPE_KEYS:
            game_state_msg.competition_type = self.COMPETITION_TYPE_KEYS[key][1]
        elif key in self.GAME_PHASE_KEYS:
            game_state_msg.game_phase = self.GAME_PHASE_KEYS[key][1]
        elif key in self.SET_PLAY_KEYS:
            game_state_msg.set_play = self.SET_PLAY_KEYS[key][1]
        elif key == "p":
            game_state_msg.penalized = not game_state_msg.penalized

            if game_state_msg.penalized:
                # Use any generic penalty type, which is not in place as they
                # all lead to the same timeout penalty and are likely handled
                # the same way
                game_state_msg.penalty = GameState.PENALTY_ILLEGAL_POSITIONING
            else:
                # If penalized is toggled off, we also cannot be penalized in place anymore
                game_state_msg.penalized_in_place = False
        elif key == "l":
            game_state_msg.penalized_in_place = not game_state_msg.penalized_in_place
            game_state_msg.penalized = game_state_msg.penalized_in_place
            game_state_msg.penalty = (
                GameState.PENALTY_MOTION_IN_SET if game_state_msg.penalized_in_place else GameState.PENALTY_NONE
            )
        elif key == "y":
            game_state_msg.has_yellow_card = not game_state_msg.has_yellow_card
            if game_state_msg.has_yellow_card:
                game_state_msg.has_red_card = False
                game_state_msg.cautions = 1
            else:
                game_state_msg.cautions = 0
        elif key == "r":
            game_state_msg.has_red_card = not game_state_msg.has_red_card
            if game_state_msg.has_red_card:
                game_state_msg.has_yellow_card = False
                game_state_msg.cautions = 2
                game_state_msg.penalized = True
                game_state_msg.penalized_in_place = False
                game_state_msg.penalty = GameState.PENALTY_SENT_OFF
            else:
                game_state_msg.cautions = 0
                game_state_msg.penalized = False
                game_state_msg.penalty = GameState.PENALTY_NONE
        elif key == "t":
            if game_state_msg.kicking_team == self.team_id:
                game_state_msg.kicking_team = self.team_id + 1
            else:
                game_state_msg.kicking_team = self.team_id
        elif key == "s":
            game_state_msg.stopped = not game_state_msg.stopped
        elif key == "+":
            game_state_msg.own_score += 1
            game_state_msg.main_state = GameState.STATE_READY
        elif key == "-":
            game_state_msg.rival_score += 1
            game_state_msg.main_state = GameState.STATE_READY

        self.sync_team_mates(game_state_msg)
        return True

    def sync_team_mates(self, game_state_msg):
        player_count = max(game_state_msg.players_per_team, self.player_number)
        player_index = self.player_number - 1

        game_state_msg.team_mates_with_penalty = [False] * player_count
        game_state_msg.team_mates_with_yellow_card = [False] * player_count
        game_state_msg.team_mates_with_red_card = [False] * player_count

        game_state_msg.team_mates_with_penalty[player_index] = game_state_msg.penalized
        game_state_msg.team_mates_with_yellow_card[player_index] = game_state_msg.has_yellow_card

    def key_mapping_info(self ) -> str:
        def build_key_mapping_string(key_mapping: dict[str, tuple[str, int]]) -> str:
            bindings = []
            for key, mapping in key_mapping.items():
                bindings.append(f"{key}: {mapping[0]} = {mapping[1]}")

            return "\n".join(bindings)

        return f"""
{build_key_mapping_string(self.STATE_KEYS)}

{build_key_mapping_string(self.COMPETITION_TYPE_KEYS)}

Set the game phase by entering:
{build_key_mapping_string(self.GAME_PHASE_KEYS)}

Set play states by entering:
{build_key_mapping_string(self.SET_PLAY_KEYS)}

p:     toggle penalized
l:     toggle in place penalty
y:     toggle yellow card
r:     toggle red card
t:     toggle kicking team
s:     toggle stopped state
+:     increase own score by 1
-:     increase rival score by 1

Competition Type:    0, Game Phase: 0
Main State:          0, Set Play:   0

Kicking Team:
Penalized:
In Place Penalized:
Yellow Card:
Red Card:
Stopped:

Goals(Own : Rival):

CTRL-C to quit"""

    def game_state_info(self, game_state_msg: GameState):
        return f"""
Competition Type:   {game_state_msg.competition_type}, Game Phase: {game_state_msg.game_phase}
Main State:         {game_state_msg.main_state}, Set Play:   {game_state_msg.set_play}

Kicking Team:       {game_state_msg.kicking_team}
Penalized:          {game_state_msg.penalized}
In Place Penalized: {game_state_msg.penalized_in_place}
Yellow Card:        {game_state_msg.has_yellow_card}
Red Card:           {game_state_msg.has_red_card}
Stopped:            {game_state_msg.stopped}

Goals(Own : Rival): {game_state_msg.own_score} : {game_state_msg.rival_score}

CTRL-C to quit"""

    def loop(self):
        game_state_msg = self.create_game_state_msg()

        try:
            print(self.key_mapping_info())

            while True:
                key = self.get_key()
                if not self.apply_key(game_state_msg, key):
                    break

                game_state_str = self.game_state_info(game_state_msg)
                # Override the lines: Each \n in the str + the extra \n from print
                line_count = game_state_str.count("\n") + 1
                sys.stdout.write("\x1b[A\x1b[K" * line_count)
                print(game_state_str)

                self.publisher.publish(game_state_msg)
        except Exception as e:
            print(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        return_key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return return_key


if __name__ == "__main__":
    rclpy.init(args=None)
    node = SimGameState()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()

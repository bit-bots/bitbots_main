"""
TeamDataCapsule
^^^^^^^^^^^^^^^
"""
import math
from collections import defaultdict

import rospy
from humanoid_league_msgs.msg import Strategy, TeamData


class TeamDataCapsule:
    def __init__(self):
        self.bot_id = rospy.get_param("bot_id", 1)
        self.strategy_sender = None  # type: rospy.Publisher
        # indexed with one to match robot ids
        self.team_data = {}
        for i in range(1, 7):
            self.team_data[i] = TeamData()
        self.team_strategy = dict()
        self.times_to_ball = dict()
        self.strategy = Strategy()
        self.strategy_update = None
        self.action_update = None
        self.role_update = None
        self.data_timeout = rospy.get_param("team_data_timeout", 2)
        self.ball_max_covariance = rospy.get_param("ball_max_covariance", 0.5)

    def is_valid(self, data: TeamData):
        return rospy.Time.now() - data.header.stamp < rospy.Duration(self.data_timeout) \
               and data.state != TeamData.STATE_PENALIZED

    def get_goalie_ball_position(self):
        """Return the ball relative to the goalie

        :return a tuple with the relative ball and the last update time
        """
        for data in self.team_data.values():
            role = data.strategy.role
            if role == Strategy.ROLE_GOALIE and self.is_valid(data):
                return data.ball_relative.pose.position.x, data.ball_relative.pose.position.y
        return None

    def get_goalie_ball_distance(self):
        """Return the distance between the goalie and the ball

        :return a tuple with the ball-goalie-distance and the last update time
        """
        goalie_ball_position = self.get_goalie_ball_position()
        if goalie_ball_position is not None:
            return math.sqrt(goalie_ball_position[0] ** 2 + goalie_ball_position[1] ** 2)
        else:
            return None

    def is_goalie_handling_ball(self):
        """ Returns true if the goalie is going to the ball."""
        for data in self.team_data.values():
            if self.is_valid(data) \
                    and data.strategy.role == Strategy.ROLE_GOALIE \
                    and data.strategy.action in [Strategy.ACTION_GOING_TO_BALL, Strategy.ACTION_KICKING]:
                return True
        return False

    def is_team_mate_kicking(self):
        """Returns true if one of the players in the own team is kicking."""
        for data in self.team_data.values():
            if self.is_valid(data) and data.strategy.action == Strategy.ACTION_KICKING:
                return True

        return False

    def team_rank_to_ball(self, own_ball_distance, count_goalies=True):
        """Returns the rank of this robot compared to the team robots concerning ball distance.
        Ignores the goalies distance, as it should not leave the goal, even if it is closer than field players.
        For example, we do not want our goalie to perform a throw in against our empty goal.

        :return the rank from 1 (nearest) to the number of robots
        """
        distances = []
        for data in self.team_data.values():
            # data should not be outdated, from a robot in play, only goalie if desired,
            # x and y covariance values should be below threshold. orientation covariance of ball does not matter
            # covariance is a 6x6 matrix as array. 0 is x 7, is y
            if self.is_valid(data) and (
                    data.strategy.role != Strategy.ROLE_GOALIE or count_goalies) \
                    and data.ball_relative.covariance[0] < self.ball_max_covariance \
                    and data.ball_relative.covariance[7] < self.ball_max_covariance:
                distances.append(math.sqrt(
                    data.ball_relative.pose.position.x ** 2 + data.ball_relative.pose.position.y ** 2))
        sorted_times = sorted(distances)
        rank = 1
        for distances in sorted_times:
            if own_ball_distance < distances:
                return rank
            rank += 1
        return rank

    def set_role(self, role):
        """Set the role of this robot in the team

        :param role: Has to be a role from humanoid_league_msgs/Strategy
        """
        assert role in [Strategy.ROLE_STRIKER, Strategy.ROLE_SUPPORTER, Strategy.ROLE_DEFENDER,
                        Strategy.ROLE_OTHER, Strategy.ROLE_GOALIE, Strategy.ROLE_IDLING]
        self.strategy.role = role
        self.role_update = rospy.get_time()

    def get_role(self):
        return self.strategy.role, self.role_update

    def set_action(self, action):
        """Set the action of this robot

        :param action: An action from humanoid_league_msgs/Strategy"""
        assert action in [Strategy.ACTION_UNDEFINED, Strategy.ACTION_POSITIONING, Strategy.ACTION_GOING_TO_BALL,
                          Strategy.ACTION_TRYING_TO_SCORE, Strategy.ACTION_WAITING, Strategy.ACTION_SEARCHING,
                          Strategy.ACTION_KICKING, Strategy.ACTION_LOCALIZING]
        self.strategy.action = action
        self.action_update = rospy.get_time()

    def get_action(self):
        return self.strategy.action, self.action_update

    def set_kickoff_strategy(self, strategy):
        assert strategy in [Strategy.SIDE_LEFT, Strategy.SIDE_MIDDLE, Strategy.SIDE_RIGHT]
        self.strategy.offensive_side = strategy
        self.strategy_update = rospy.get_time()

    def get_kickoff_strategy(self):
        return self.strategy.offensive_side, self.strategy_update

    def team_data_callback(self, msg):
        self.team_data[msg.robot_id] = msg

    def publish_strategy(self):
        """Publish for team comm"""
        self.strategy_sender.publish(self.strategy)

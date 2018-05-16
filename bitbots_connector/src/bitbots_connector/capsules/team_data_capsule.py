"""
TeamDataCapsule
^^^^^^^^^^^^^^^
"""
import math

import rospy
from humanoid_league_msgs.msg import Strategy, TeamData


class TeamDataCapsule:
    def __init__(self):
        self.strategy_sender = None  # type: rospy.Publisher
        self.team_data = None
        self.strategy = Strategy()
        self.last_update_team_data = None
        self.strategy_update = None
        self.action_update = None
        self.role_update = None

    def get_team_goalie_ball_position(self):
        """Return the ball relative to the goalie

        :return a tuple with the relative ball and the last update time
        """
        i = 0
        for state in self.team_data.state:
            if state is TeamData.ROLE_GOALIE:
                return (self.team_data.ball_relative[i].x, self.team_data.ball_relative[i].y), self.last_update_team_data
            i += 1
        return None

    def get_goalie_ball_distance(self):
        """Return the distance between the goalie and the ball

        :return a tuple with the ball-goalie-distance and the last update time
        """
        goalie_ball_position = self.get_team_goalie_ball_position()
        if goalie_ball_position is not None:
            return math.sqrt(goalie_ball_position[0] ** 2 + goalie_ball_position[1] ** 2)
        else:
            return None

    def team_rank_to_ball(self, own_ball_relative):
        """Returns the rank of this robot compared to the team robots concerning ball distance

        :param own_ball_relative: the ball relative to this robot
        :return the rank from 1 (nearest) to the number of robots
        """
        own_distance = math.sqrt(own_ball_relative[0] ** 2 + own_ball_relative[1] ** 2)
        relative_balls = self.team_data.ball_relative
        ball_distances = [math.sqrt(ball.x ** 2 + ball.y ** 2) for ball in relative_balls]
        ball_distances.sort()
        rank = 1
        for distance in ball_distances:
            if own_distance < distance:
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
        self.strategy_sender.publish(self.strategy)
        self.role_update = rospy.get_time()

    def get_role(self):
        return self.strategy.role, self.role_update

    def set_action(self, action):
        """Set the action of this robot

        :param action: An action from humanoid_league_msgs/Strategy"""
        assert action in [TeamData.ACTION_UNDEFINED, TeamData.ACTION_POSITIONING, TeamData.ACTION_GOING_TO_BALL,
                          TeamData.ACTION_TRYING_TO_SCORE, TeamData.ACTION_WAITING]
        self.strategy.action = action
        self.strategy_sender.publish(self.strategy)
        self.action_update = rospy.get_time()

    def get_action(self):
        return self.strategy.action, self.action_update

    def publish_kickoff_strategy(self, strategy):
        """Set the kickoff strategy"""
        assert strategy in [Strategy.SIDE_LEFT, Strategy.SIDE_MIDDLE, Strategy.SIDE_RIGHT]
        self.strategy.offensive_side = strategy
        self.strategy_sender.publish(self.strategy)
        self.strategy_update = rospy.get_time()

    def get_kickoff_strategy(self):
        return self.strategy.offensive_side, self.strategy_update

    def team_data_callback(self, msg):
        self.team_data = msg
        self.last_update_team_data = rospy.get_time()

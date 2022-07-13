"""
TeamDataCapsule
^^^^^^^^^^^^^^^
"""
import math
from typing import Dict, Union

from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from humanoid_league_msgs.msg import Strategy, TeamData
from geometry_msgs.msg import PointStamped
from bitbots_utils.utils import get_parameters_from_other_node
from std_msgs.msg import Float32

class TeamDataCapsule:

    def __init__(self, node: Node):
        self.node = node

        # Retrieve game settings from parameter blackboard
        params = get_parameters_from_other_node(self.node, 'parameter_blackboard', ['bot_id', 'role'])
        self.bot_id = params['bot_id']
        role_name = params['role']

        self.strategy_sender = None
        self.time_to_ball_publisher = None
        # indexed with one to match robot ids
        self.team_data : Dict[TeamData] = {}
        for i in range(1, 7):
            self.team_data[i] = TeamData()
        self.team_strategy = dict()
        self.times_to_ball = dict()
        self.roles = {
            'striker': Strategy.ROLE_STRIKER,
            'offense': Strategy.ROLE_STRIKER,
            'supporter': Strategy.ROLE_SUPPORTER,
            'defender': Strategy.ROLE_DEFENDER,
            'defense': Strategy.ROLE_DEFENDER,
            'other': Strategy.ROLE_OTHER,
            'goalie': Strategy.ROLE_GOALIE,
            'idle': Strategy.ROLE_IDLING
        }
        self.own_time_to_ball = 9999.0
        self.strategy = Strategy()
        self.strategy.role = self.roles[role_name]
        self.strategy_update = None
        self.action_update = None
        self.role_update = None
        self.data_timeout = self.node.get_parameter("team_data_timeout").get_parameter_value().double_value
        self.ball_max_covariance = self.node.get_parameter("ball_max_covariance").get_parameter_value().double_value
        self.ball_lost_time = Duration(
            seconds=self.node.get_parameter('body.ball_lost_time').get_parameter_value().double_value)
        self.pose_precision_threshold_x_sdev = self.node.get_parameter(
            'body.pose_precision_threshold.x_sdev').get_parameter_value().double_value
        self.pose_precision_threshold_y_sdev = self.node.get_parameter(
            'body.pose_precision_threshold.y_sdev').get_parameter_value().double_value
        self.pose_precision_threshold_theta_sdev = self.node.get_parameter(
            'body.pose_precision_threshold.theta_sdev').get_parameter_value().double_value

    def is_valid(self, data: TeamData):
        return self.node.get_clock().now() - Time.from_msg(data.header.stamp) < Duration(seconds=self.data_timeout) \
               and data.state != TeamData.STATE_PENALIZED

    def get_goalie_ball_position(self):
        """Return the ball relative to the goalie

        :return a tuple with the relative ball and the last update time
        """
        data: TeamData
        for data in self.team_data.values():
            role = data.strategy.role
            if role == Strategy.ROLE_GOALIE and self.is_valid(data):
                return data.ball_relative.pose.position.x, data.ball_relative.pose.position.y  # TODO fiels are missing
        return None

    def get_goalie_ball_distance(self):
        """Return the distance between the goalie and the ball

        :return a tuple with the ball-goalie-distance and the last update time
        """
        goalie_ball_position = self.get_goalie_ball_position()
        if goalie_ball_position is not None:
            return math.sqrt(goalie_ball_position[0]**2 + goalie_ball_position[1]**2)
        else:
            return None

    def is_goalie_handling_ball(self):
        """ Returns true if the goalie is going to the ball."""
        data: TeamData
        for data in self.team_data.values():
            if self.is_valid(data) \
                    and data.strategy.role == Strategy.ROLE_GOALIE \
                    and data.strategy.action in [Strategy.ACTION_GOING_TO_BALL, Strategy.ACTION_KICKING]:
                return True
        return False

    def is_team_mate_kicking(self):
        """Returns true if one of the players in the own team is kicking."""
        data: TeamData
        for data in self.team_data.values():
            if self.is_valid(data) and data.strategy.action == Strategy.ACTION_KICKING:
                return True

        return False

    def team_rank_to_ball(self, own_ball_distance, count_goalies=True, use_time_to_ball=False):
        """Returns the rank of this robot compared to the team robots concerning ball distance.
        Ignores the goalies distance, as it should not leave the goal, even if it is closer than field players.
        For example, we do not want our goalie to perform a throw in against our empty goal.

        :return the rank from 1 (nearest) to the number of robots
        """
        distances = []
        data: TeamData
        for data in self.team_data.values():
            # data should not be outdated, from a robot in play, only goalie if desired,
            # x and y covariance values should be below threshold. orientation covariance of ball does not matter
            # covariance is a 6x6 matrix as array. 0 is x, 7 is y
            if self.is_valid(data) and (
                    data.strategy.role != Strategy.ROLE_GOALIE or count_goalies) \
                    and data.ball_absolute.covariance[0] < self.ball_max_covariance \
                    and data.ball_absolute.covariance[7] < self.ball_max_covariance:
                if use_time_to_ball:
                    distances.append(data.time_to_position_at_ball)
                else:
                    distances.append(self.get_robot_ball_euclidean_distance(data))
        for rank, distance in enumerate(sorted(distances)):
            if own_ball_distance < distance:
                return rank + 1
        return len(distances) + 1

    def get_robot_ball_euclidean_distance(self, robot_teamdata: TeamData):
        ball_rel_x = robot_teamdata.ball_absolute.pose.position.x - robot_teamdata.robot_position.pose.position.x
        ball_rel_y = robot_teamdata.ball_absolute.pose.position.y - robot_teamdata.robot_position.pose.position.y
        dist = math.sqrt(ball_rel_x**2 + ball_rel_y**2)
        return dist

    def set_role(self, role):
        """Set the role of this robot in the team

        :param role: Has to be a role from humanoid_league_msgs/Strategy
        """
        assert role in [
            Strategy.ROLE_STRIKER, Strategy.ROLE_SUPPORTER, Strategy.ROLE_DEFENDER, Strategy.ROLE_OTHER,
            Strategy.ROLE_GOALIE, Strategy.ROLE_IDLING
        ]
        self.strategy.role = role
        self.role_update = float(self.node.get_clock().now().seconds_nanoseconds()[0] +
                                 self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9)

    def get_role(self):
        return self.strategy.role, self.role_update

    def set_action(self, action):
        """Set the action of this robot

        :param action: An action from humanoid_league_msgs/Strategy"""
        assert action in [
            Strategy.ACTION_UNDEFINED, Strategy.ACTION_POSITIONING, Strategy.ACTION_GOING_TO_BALL,
            Strategy.ACTION_TRYING_TO_SCORE, Strategy.ACTION_WAITING, Strategy.ACTION_SEARCHING,
            Strategy.ACTION_KICKING, Strategy.ACTION_LOCALIZING
        ]
        self.strategy.action = action
        self.action_update = float(self.node.get_clock().now().seconds_nanoseconds()[0] +
                                   self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9)

    def get_action(self):
        return self.strategy.action, self.action_update

    def set_kickoff_strategy(self, strategy):
        assert strategy in [Strategy.SIDE_LEFT, Strategy.SIDE_MIDDLE, Strategy.SIDE_RIGHT]
        self.strategy.offensive_side = strategy
        self.strategy_update = float(self.node.get_clock().now().seconds_nanoseconds()[0] +
                                     self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9)

    def get_kickoff_strategy(self):
        return self.strategy.offensive_side, self.strategy_update

    def get_active_teammate_poses(self, count_goalies=False):
        """ Returns the poses of all playing robots """
        poses = []
        data: TeamData
        for data in self.team_data.values():
            if self.is_valid(data) and (data.strategy.role != Strategy.ROLE_GOALIE or count_goalies):
                poses.append(data.robot_position.pose)
        return poses

    def get_own_time_to_ball(self):
        return self.own_time_to_ball

    def team_data_callback(self, msg: TeamData):
        # Save team data
        self.team_data[msg.robot_id] = msg

    def publish_strategy(self):
        """Publish for team comm"""
        self.strategy_sender.publish(self.strategy)

    def publish_time_to_ball(self):
        self.time_to_ball_publisher.publish(Float32(data=self.own_time_to_ball))

    def get_teammate_ball_seen_time(self):
        """Returns the time at which a teammate has seen the ball accurately enough"""
        teammate_ball = self.get_teammate_ball()
        if teammate_ball is not None:
            return teammate_ball.header.stamp
        else:
            return Time(seconds=0, nanoseconds=0, clock_type=ClockType.ROS_TIME)

    def teammate_ball_is_valid(self):
        """Returns true if a teammate has seen the ball accurately enough"""
        return self.get_teammate_ball() is not None

    def get_teammate_ball(self):
        """Returns the ball from the closest teammate that has accurate enough localization and ball precision"""

        def std_dev_from_covariance(covariance):
            x_sdev = covariance[0]  # position 0,0 in a 6x6-matrix
            y_sdev = covariance[7]  # position 1,1 in a 6x6-matrix
            theta_sdev = covariance[35]  # position 5,5 in a 6x6-matrix
            return x_sdev, y_sdev, theta_sdev

        best_robot_dist = 9999
        best_ball = None

        teamdata: TeamData
        for teamdata in self.team_data.values():
            if not self.is_valid(teamdata):
                continue
            ball = teamdata.ball_absolute
            ball_x_std_dev, ball_y_std_dev, _ = std_dev_from_covariance(ball.covariance)
            robot = teamdata.robot_position
            robot_x_std_dev, robot_y_std_dev, robot_theta_std_dev = std_dev_from_covariance(robot.covariance)
            stamp = teamdata.header.stamp
            if self.get_clock().now() - Time.from_msg(stamp) < self.ball_lost_time:
                if ball_x_std_dev < self.ball_max_covariance and ball_y_std_dev < self.ball_max_covariance:
                    if robot_x_std_dev < self.pose_precision_threshold_x_sdev and \
                            robot_y_std_dev < self.pose_precision_threshold_y_sdev and \
                            robot_theta_std_dev < self.pose_precision_threshold_theta_sdev:
                        robot_dist = self.get_robot_ball_euclidean_distance(teamdata)
                        if robot_dist < best_robot_dist:
                            best_ball = PointStamped()
                            best_ball.header = teamdata.header
                            best_ball.point.x = teamdata.ball_absolute.pose.position.x
                            best_ball.point.y = teamdata.ball_absolute.pose.position.y
                            best_robot_dist = robot_dist
        return best_ball

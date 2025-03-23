from typing import Literal, Optional

import numpy as np
from bitbots_utils.utils import get_parameters_from_other_node
from geometry_msgs.msg import PointStamped, Pose
from rclpy.duration import Duration
from rclpy.time import Time
from ros2_numpy import numpify
from std_msgs.msg import Float32

from bitbots_blackboard.capsules import AbstractBlackboardCapsule
from bitbots_msgs.msg import Strategy, TeamData


class TeamDataCapsule(AbstractBlackboardCapsule):
    def __init__(self, node, blackboard):
        """Handles incoming team data communication."""
        super().__init__(node, blackboard)

        # Publishers
        self.strategy_sender = self._node.create_publisher(Strategy, "strategy", 2)
        self.time_to_ball_publisher = self._node.create_publisher(Float32, "time_to_ball", 2)

        # Retrieve game settings from parameter blackboard
        params = get_parameters_from_other_node(self._node, "parameter_blackboard", ["role"])
        self.role = params["role"]

        # Data
        # indexed with one to match robot ids
        self.team_data: dict[int, TeamData] = {}
        for i in range(1, 7):
            self.team_data[i] = TeamData()
        self.times_to_ball = dict()
        self.own_time_to_ball = 9999.0
        #Hier Subscibtion hinzufügen
        self.own_time_to_goal = 9999.0

        # Mapping
        self.roles_mapping = {
            "striker": Strategy.ROLE_STRIKER,
            "offense": Strategy.ROLE_STRIKER,
            "supporter": Strategy.ROLE_SUPPORTER,
            "defender": Strategy.ROLE_DEFENDER,
            "defense": Strategy.ROLE_DEFENDER,
            "other": Strategy.ROLE_OTHER,
            "goalie": Strategy.ROLE_GOALIE,
            "idle": Strategy.ROLE_IDLING,
        }

        # Possible actions
        self.actions = {
            Strategy.ACTION_UNDEFINED,
            Strategy.ACTION_POSITIONING,
            Strategy.ACTION_GOING_TO_BALL,
            Strategy.ACTION_TRYING_TO_SCORE,
            Strategy.ACTION_WAITING,
            Strategy.ACTION_SEARCHING,
            Strategy.ACTION_KICKING,
            Strategy.ACTION_LOCALIZING,
        }

        # The strategy which is communicated to the other robots
        self.strategy = Strategy()
        self.strategy.role = self.roles_mapping[self.role]
        self.role_update: float = 0.0
        self.strategy_update: float = 0.0
        self.action_update: float = 0.0

        # Config
        self.data_timeout: float = float(self._node.get_parameter("team_data_timeout").value)
        self.ball_max_covariance: float = float(self._node.get_parameter("ball_max_covariance").value)

    def is_valid(self, data: TeamData) -> bool:
        """
        Checks if a team data message from a given robot is valid.
        Meaning it is not too old and the robot is not penalized.
        """
        return (
            self._node.get_clock().now() - Time.from_msg(data.header.stamp) < Duration(seconds=self.data_timeout)
            and data.state != TeamData.STATE_PENALIZED
        )

    def is_goalie_handling_ball(self) -> bool:
        """Returns true if the goalie is going to the ball."""
        data: TeamData
        for data in self.team_data.values():
            if (
                self.is_valid(data)
                and data.strategy.role == Strategy.ROLE_GOALIE
                and data.strategy.action in [Strategy.ACTION_GOING_TO_BALL, Strategy.ACTION_KICKING]
            ):
                return True
        return False

    def is_team_mate_kicking(self) -> bool:
        """Returns true if one of the players in the own team is kicking."""
        data: TeamData
        for data in self.team_data.values():
            if self.is_valid(data) and data.strategy.action == Strategy.ACTION_KICKING:
                return True
        return False

    def team_rank_to_ball(
        self, own_ball_distance: float, count_goalies: bool = True, use_time_to_ball: bool = False
    ) -> int:
        """
        Returns the rank of this robot compared to the team robots concerning ball distance.

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
            if (
                self.is_valid(data)
                and (data.strategy.role != Strategy.ROLE_GOALIE or count_goalies)
                and data.ball_absolute.covariance[0] < self.ball_max_covariance
                and data.ball_absolute.covariance[7] < self.ball_max_covariance
            ):
                if use_time_to_ball:
                    distances.append(data.time_to_position_at_ball)
                else:
                    distances.append(
                        np.linalg.norm(
                            numpify(data.ball_absolute.pose.position) - numpify(data.robot_position.pose.position)
                        )
                    )
        for rank, distance in enumerate(sorted(distances)):
            if own_ball_distance < distance:
                return rank + 1
        return len(distances) + 1
    
    def team_rank_to_goal(
        self, own_goal_distance: float, count_goalies: bool = True, use_time_to_goal: bool = False
    ) -> int:
        """
        Returns the rank of this robot compared to the team robots concerning the distance to the own goal.

        Ignores the goalies distance, as it should be used i situations where the goalie is handling the ball in the offensive

        :return the rank from 1 (nearest) to the number of robots
        """
        distances = []
        data: TeamData
        for data in self.team_data.values():
            # data should not be outdated, from a robot in play, only goalie if desired,
            if (
                self.is_valid(data)
                and (data.strategy.role != Strategy.ROLE_GOALIE or count_goalies)
            ):
                if use_time_to_ball:
                    distances.append(data.time_to_position_at_ball)
                else:
                    distances.append(
                        np.linalg.norm(
                            numpify(self._blackboard.world_model.get_map_based_own_goal_center_xy()) - numpify(data.robot_position.pose.position)
                        )
                    )
        for rank, distance in enumerate(sorted(distances)):
            if own_goal_distance < distance:
                return rank + 1
        return len(distances) + 1
   
    def set_action(self, action: int) -> None:
        """Set the action of this robot

        :param action: An action from bitbots_msgs/Strategy"""
        assert action in self.actions
        self.strategy.action = action
        self.action_update = self._node.get_clock().now().nanoseconds / 1e9

    def get_action(self) -> tuple[int, float]:
        return self.strategy.action, self.action_update

    def set_role(self, role: Literal["goalie", "offense", "defense"]) -> None:
        """Set the role of this robot in the team

        :param role: String describing the role.
        """
        self.role = role
        self.strategy.role = self.roles_mapping[role]
        self.role_update = self._node.get_clock().now().nanoseconds / 1e9

    def get_role(self) -> tuple[int, float]:
        return self.strategy.role, self.role_update

    def set_kickoff_strategy(
        self,
        strategy: Literal[Strategy.SIDE_LEFT, Strategy.SIDE_MIDDLE, Strategy.SIDE_RIGHT],  # type: ignore[valid-type]
    ) -> None:
        self.strategy.offensive_side = strategy
        self.strategy_update = self._node.get_clock().now().nanoseconds / 1e9

    def get_kickoff_strategy(self) -> tuple[int, float]:
        return self.strategy.offensive_side, self.strategy_update

    def get_active_teammate_poses(self, count_goalies: bool = False) -> list[Pose]:
        """Returns the poses of all playing robots"""
        poses = []
        data: TeamData
        for data in self.team_data.values():
            if self.is_valid(data) and (data.strategy.role != Strategy.ROLE_GOALIE or count_goalies):
                poses.append(data.robot_position.pose)
        return poses

    def get_number_of_active_field_players(self, count_goalie: bool = False) -> int:
        def is_not_goalie(team_data: TeamData) -> bool:
            return team_data.strategy.role != Strategy.ROLE_GOALIE

        # Get the team data infos for all robots (ignoring the robot id/name)
        team_data_infos = self.team_data.values()

        # Remove goalie data if needed
        if not count_goalie:
            team_data_infos = filter(is_not_goalie, team_data_infos)  # type: ignore[assignment]

        # Count valid team data infos (aka robots with valid team data)
        return sum(map(self.is_valid, team_data_infos))

    def get_is_goalie_active(self) -> bool:
        def is_a_goalie(team_data: TeamData) -> bool:
            return team_data.strategy.role == Strategy.ROLE_GOALIE

        # Get the team data infos for all robots (ignoring the robot id/name)
        team_data_infos = self.team_data.values()  # type: ignore[assignment]

        # Remove none goalie Data
        team_data_infos = filter(is_a_goalie, team_data_infos)  # type: ignore[assignment]

        # Count valid team data infos (aka robots with valid team data)
        return sum(map(self.is_valid, team_data_infos)) == 1

    def get_own_time_to_ball(self) -> float:
        return self.own_time_to_ball

    def team_data_callback(self, msg: TeamData):
        # Save team data
        self.team_data[msg.robot_id] = msg

    def publish_strategy(self) -> None:
        """Publish for team comm"""
        self.strategy_sender.publish(self.strategy)

    def publish_time_to_ball(self):
        self.time_to_ball_publisher.publish(Float32(data=self.own_time_to_ball))

    def teammate_ball_is_valid(self) -> bool:
        """Returns true if a teammate has seen the ball accurately enough"""
        return self.get_teammate_ball() is not None

    def get_teammate_ball(self) -> Optional[PointStamped]:
        """Returns the best ball from all teammates that satisfies a minimum ball precision"""

        # Get the team data infos for all valid robots (ignoring the robot id)
        team_data_infos = filter(self.is_valid, self.team_data.values())

        def is_ball_good_enough(team_data: TeamData) -> bool:
            return bool(
                team_data.ball_absolute.covariance[0] < self.ball_max_covariance
                and team_data.ball_absolute.covariance[7] < self.ball_max_covariance
            )

        # Filter robots with too high ball covariance
        team_data_infos = filter(is_ball_good_enough, team_data_infos)

        def get_ball_max_covariance(team_data: TeamData) -> float:
            return max(team_data.ball_absolute.covariance[0], team_data.ball_absolute.covariance[7])

        # Get robot with lowest maximum ball covariance
        team_data_with_best_ball = min(team_data_infos, key=get_ball_max_covariance, default=None)

        # Convert ball to PointStamped if a ball was found
        if team_data_with_best_ball is not None:
            return PointStamped(
                header=team_data_with_best_ball.header, point=team_data_with_best_ball.ball_absolute.pose.position
            )
        return None

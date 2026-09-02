from typing import Literal

from bitbots_utils.utils import get_parameters_from_other_node
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Float32

from bitbots_blackboard.capsules import AbstractBlackboardCapsule, cached_capsule_function
from bitbots_msgs.msg import Strategy, TeamData

# Time to ball that is reported by a robot which does not know where the ball is
TIME_TO_BALL_UNKNOWN = 9999.0


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
        self.own_time_to_ball = TIME_TO_BALL_UNKNOWN
        self.last_time_team_mate_kicked = None

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
            Strategy.ACTION_PASSIVE,
        }

        # The strategy which is communicated to the other robots
        self.strategy = Strategy()
        self.strategy.role = self.roles_mapping[self.role]
        self.role_update: float = 0.0
        self.strategy_update: float = 0.0
        self.action_update: float = 0.0

        # Config
        self.data_timeout: float = float(self._node.get_parameter("team_data_timeout").value)

    @cached_capsule_function
    def time(self) -> Time:
        """Returns the current time of the node, this is its own function so it can be cached during the decision loop."""
        return self._node.get_clock().now()

    def is_valid(self, data: TeamData) -> bool:
        """
        Checks if a team data message from a given robot is valid.
        Meaning it is not too old and the robot is not penalized.
        """
        return (
            self.time() - Time.from_msg(data.header.stamp) < Duration(seconds=self.data_timeout)
            and self.time().nanoseconds / 1e9 > self.data_timeout  # Handle edge case at simulation start
            and data.state != TeamData.STATE_PENALIZED
        )

    @cached_capsule_function
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

    @cached_capsule_function
    def is_team_mate_kicking(self) -> bool:
        """Returns true if one of the players in the own team is kicking."""
        data: TeamData
        for data in self.team_data.values():
            if self.is_valid(data) and data.strategy.action == Strategy.ACTION_KICKING:
                return True
        return False

    def team_rank_to_ball(self, own_time_to_ball: float, count_goalies: bool = True) -> int:
        """
        Returns the rank of this robot compared to the team robots concerning the time it takes
        them to reach the ball. Every robot estimates this time based on its own team ball, so all
        robots that know where the ball is take part in this comparison, no matter if they observed
        the ball themselves or if a teammate told them about it.

        If count_goalies is False, the goalie is ignored, as it should not leave the goal,
        even if it is closer than field players.
        For example, we do not want our goalie to perform a throw in against our empty goal.

        :return the rank from 1 (nearest) to the number of robots
        """
        times_to_ball = []
        data: TeamData
        for data in self.team_data.values():
            # data should not be outdated, from a robot in play, only goalie if desired,
            # and the robot needs to know where the ball is
            if (
                self.is_valid(data)
                and (data.strategy.role != Strategy.ROLE_GOALIE or count_goalies)
                and data.strategy.action != Strategy.ACTION_PASSIVE
                and data.time_to_position_at_ball < TIME_TO_BALL_UNKNOWN
            ):
                times_to_ball.append(data.time_to_position_at_ball)
        for rank, time_to_ball in enumerate(sorted(times_to_ball)):
            if own_time_to_ball < time_to_ball:
                return rank + 1
        return len(times_to_ball) + 1

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

    @cached_capsule_function
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

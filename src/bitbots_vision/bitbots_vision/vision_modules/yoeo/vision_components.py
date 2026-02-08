from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from soccer_vision_2d_msgs.msg import BallArray, GoalpostArray, Robot, RobotArray
from std_msgs.msg import Header

from bitbots_vision.vision_modules import candidate, debug, ros_utils

from . import yoeo_handlers

logger = rclpy.logging.get_logger("yoeo_vision_components")


class AbstractVisionComponent(ABC):
    """
    Abstract class for the YOEO vision components.
    """

    def __init__(
        self, node: Node, yoeo_handler: yoeo_handlers.IYOEOHandler, debug_image: debug.DebugImage, config: dict
    ):
        self._node = node
        self._yoeo_handler = yoeo_handler
        self._debug_image = debug_image
        self._config = config

    @abstractmethod
    def run(self, image: np.ndarray, header: Header) -> None:
        """
        This method runs the vision component.
        """
        ...


class YOEOComponent(AbstractVisionComponent):
    """
    This component runs the YOEO network. It is MANDATORY and must be run BEFORE any of the other components.
    """

    def run(self, image: np.ndarray, header: Header) -> None:
        self._yoeo_handler.set_image(image)
        self._yoeo_handler.predict()


class BallDetectionComponent(AbstractVisionComponent):
    """
    This component carries out the ball detection using YOEO.
    """

    def __init__(
        self, node: Node, yoeo_handler: yoeo_handlers.IYOEOHandler, debug_image: debug.DebugImage, config: dict
    ):
        super().__init__(node, yoeo_handler, debug_image, config)

        self._publisher = self._node.create_publisher(BallArray, self._config["ROS_ball_msg_topic"], qos_profile=1)

    def run(self, image: np.ndarray, header: Header) -> None:
        # Get all ball candidates from YOEO
        candidates = self._yoeo_handler.get_detection_candidates_for("ball")

        # Filter candidates by rating and count
        candidates = candidate.Candidate.sort_candidates(candidates)
        top_candidates = candidates[: self._config["ball_candidate_max_count"]]
        final_candidates = candidate.Candidate.rating_threshold(
            top_candidates, self._config["ball_candidate_rating_threshold"]
        )

        # Publish ball candidates
        self._publish_balls_message(header, final_candidates)

        # Draw candidates on debug image
        self._debug_image.draw_ball_candidates(candidates, DebugImageComponent.Colors.ball, thickness=1)
        self._debug_image.draw_ball_candidates(final_candidates, DebugImageComponent.Colors.ball, thickness=3)

    def _publish_balls_message(self, header: Header, candidates: list[candidate.Candidate]) -> None:
        ball_messages = list(map(ros_utils.build_ball_msg, candidates))
        balls_message = ros_utils.build_ball_array_msg(header, ball_messages)
        assert self._publisher is not None, "Publisher not set!"
        self._publisher.publish(balls_message)


class GoalpostDetectionComponent(AbstractVisionComponent):
    """
    This component carries out the goalpost detection using YOEO.
    """

    def __init__(
        self, node: Node, yoeo_handler: yoeo_handlers.IYOEOHandler, debug_image: debug.DebugImage, config: dict
    ):
        super().__init__(node, yoeo_handler, debug_image, config)

        self._publisher = self._node.create_publisher(
            GoalpostArray, self._config["ROS_goal_posts_msg_topic"], qos_profile=1
        )

    def run(self, image: np.ndarray, header: Header) -> None:
        # Get all goalpost candidates from YOEO
        candidates = self._yoeo_handler.get_detection_candidates_for("goalpost")

        # Publish goalpost candidates
        self._publish_goalposts_message(header, candidates)

        # Draw candidates on debug image
        self._debug_image.draw_robot_candidates(candidates, DebugImageComponent.Colors.goalposts, thickness=1)
        self._debug_image.draw_robot_candidates(candidates, DebugImageComponent.Colors.goalposts, thickness=3)

    def _publish_goalposts_message(self, header: Header, candidates: list[candidate.Candidate]) -> None:
        goalpost_messages = [ros_utils.build_goal_post_msg(candidate) for candidate in candidates]
        goalposts_message = ros_utils.build_goal_post_array_msg(header, goalpost_messages)
        assert self._publisher is not None, "Publisher not set!"
        self._publisher.publish(goalposts_message)


class LineDetectionComponent(AbstractVisionComponent):
    """
    This component carries out the line detection using YOEO.
    """

    def __init__(
        self, node: Node, yoeo_handler: yoeo_handlers.IYOEOHandler, debug_image: debug.DebugImage, config: dict
    ):
        super().__init__(node, yoeo_handler, debug_image, config)
        self._publisher = self._node.create_publisher(Image, self._config["ROS_line_mask_msg_topic"], qos_profile=1)

    def run(self, image: np.ndarray, header: Header) -> None:
        # Get line mask from YOEO
        line_mask = (self._yoeo_handler.get_segmentation_mask_for("lines")) * 255

        # Publish line mask
        self._publish_line_mask_msg(header, line_mask)

        # Draw line mask on debug image
        self._debug_image.draw_mask(line_mask, color=DebugImageComponent.Colors.lines)

    def _publish_line_mask_msg(self, header: Header, line_mask: np.ndarray) -> None:
        line_mask_msg = ros_utils.build_image_msg(header, line_mask, "8UC1")
        assert self._publisher is not None, "Publisher not set!"
        self._publisher.publish(line_mask_msg)


class FieldDetectionComponent(AbstractVisionComponent):
    """
    This component carries out the field detection using YOEO.
    """

    def __init__(
        self, node: Node, yoeo_handler: yoeo_handlers.IYOEOHandler, debug_image: debug.DebugImage, config: dict
    ):
        super().__init__(node, yoeo_handler, debug_image, config)
        self._publisher = self._node.create_publisher(
            Image, self._config["ROS_field_mask_image_msg_topic"], qos_profile=1
        )

    def run(self, image: np.ndarray, header: Header) -> None:
        # Get field mask from YOEO
        field_mask = (self._yoeo_handler.get_segmentation_mask_for("field")) * 255

        # Publish field mask
        self._publish_field_mask_msg(header, field_mask)

    def _publish_field_mask_msg(self, header: Header, field_mask: np.ndarray) -> None:
        field_mask_msg = ros_utils.build_image_msg(header, field_mask, "8UC1")
        assert self._publisher is not None, "Publisher not set!"
        self._publisher.publish(field_mask_msg)


class RobotDetectionComponent(AbstractVisionComponent):
    """
    This component carries out the robot detection using YOEO with team color detection done by YOEO.
    """

    def __init__(
        self,
        node: Node,
        yoeo_handler: yoeo_handlers.IYOEOHandler,
        debug_image: debug.DebugImage,
        config: dict,
        team_color_detection_supported: bool,
    ):
        super().__init__(node, yoeo_handler, debug_image, config)

        self._team_color_detection_supported = team_color_detection_supported
        self._publisher = self._node.create_publisher(RobotArray, self._config["ROS_robot_msg_topic"], qos_profile=1)

    def run(self, image: np.ndarray, header: Header) -> None:
        robot_msgs: list[Robot] = []
        own_color = ros_utils.get_robot_color_for_team(Robot().attributes.TEAM_OWN)
        assert own_color != ros_utils.RobotColor.UNKNOWN
        opponent_color = ros_utils.get_robot_color_for_team(Robot().attributes.TEAM_OPPONENT)

        if self._team_color_detection_supported:
            # add team mates to robots_msgs
            team_mate_candidates = self.get_candidates(own_color)
            team_mate_candidate_messages = self._create_robots_messages(
                Robot().attributes.TEAM_OWN, team_mate_candidates
            )
            robot_msgs.extend(team_mate_candidate_messages)

            # add opponents to robots_msgs
            opponent_candidates = self.get_candidates(opponent_color)
            opponent_candidate_messages = self._create_robots_messages(
                Robot().attributes.TEAM_OPPONENT, opponent_candidates
            )
            robot_msgs.extend(opponent_candidate_messages)

            # add remaining robots to robots_msgs
            remaining_candidates = self.get_candidates(ros_utils.RobotColor.UNKNOWN)
            remaining_candidate_messages = self._create_robots_messages(
                Robot().attributes.TEAM_UNKNOWN, remaining_candidates
            )
            robot_msgs.extend(remaining_candidate_messages)
        else:
            robot_candidates = self._yoeo_handler.get_detection_candidates_for("robot")
            robot_candidate_messages = [
                ros_utils.build_robot_msg(robot_candidate) for robot_candidate in robot_candidates
            ]
            robot_msgs.extend(robot_candidate_messages)

        self._publish_robots_message(header, robot_msgs)

        if self._team_color_detection_supported:
            self._debug_image.draw_robot_candidates(
                team_mate_candidates, DebugImageComponent.Colors.robot_team_mates, thickness=3
            )
            self._debug_image.draw_robot_candidates(
                opponent_candidates, DebugImageComponent.Colors.robot_opponents, thickness=3
            )
            self._debug_image.draw_robot_candidates(
                remaining_candidates, DebugImageComponent.Colors.robot_unknown, thickness=3
            )
        else:
            self._debug_image.draw_robot_candidates(
                robot_candidates, DebugImageComponent.Colors.robot_unknown, thickness=3
            )

    def get_candidates(self, team_color: ros_utils.RobotColor) -> list[candidate.Candidate]:
        candidates: list[candidate.Candidate] = []
        if team_color == ros_utils.RobotColor.BLUE:
            candidates = self._yoeo_handler.get_detection_candidates_for("robot_blue")
        elif team_color == ros_utils.RobotColor.RED:
            candidates = self._yoeo_handler.get_detection_candidates_for("robot_red")
        else:
            candidates = self._yoeo_handler.get_detection_candidates_for("robot_unknown")
        return candidates

    @staticmethod
    def _create_robots_messages(
        team: ros_utils.T_RobotAttributes_Team, candidates: list[candidate.Candidate]
    ) -> list[Robot]:
        return [ros_utils.build_robot_msg(robot_candidate, team) for robot_candidate in candidates]

    def _publish_robots_message(self, header: Header, robot_msgs: list[Robot]) -> None:
        robots_msg = ros_utils.build_robot_array_msg(header, robot_msgs)
        assert self._publisher is not None, "Publisher not set!"
        self._publisher.publish(robots_msg)


class DebugImageComponent(AbstractVisionComponent):
    """
    This component handles and publishes the debug image.
    """

    class Colors:
        # BGR
        ball = (0, 255, 0)  # green
        robot_team_mates = (255, 255, 102)  # cyan
        robot_opponents = (153, 51, 255)  # magenta
        robot_unknown = (160, 160, 160)  # grey
        team_mates = (153, 51, 255)  # magenta
        opponents = (255, 255, 102)  # cyan
        unknown_obstacles = (160, 160, 160)  # grey
        goalposts = (255, 255, 255)  # white
        lines = (255, 0, 0)  # blue

    def __init__(
        self, node: Node, yoeo_handler: yoeo_handlers.IYOEOHandler, debug_image: debug.DebugImage, config: dict
    ):
        super().__init__(node, yoeo_handler, debug_image, config)

        self._publisher = self._node.create_publisher(Image, self._config["ROS_debug_image_msg_topic"], qos_profile=1)

    def run(self, image: np.ndarray, header: Header) -> None:
        debug_image_msg = ros_utils.build_image_msg(header, self._debug_image.get_image(), "bgr8")
        self._publisher.publish(debug_image_msg)

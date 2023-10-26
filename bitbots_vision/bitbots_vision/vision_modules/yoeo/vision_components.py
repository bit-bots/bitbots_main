from __future__ import annotations

import numpy as np
import rclpy

from abc import ABC, abstractmethod
from rclpy.node import Node
from sensor_msgs.msg import Image
from typing import Dict, Optional, List, Tuple

from humanoid_league_msgs.msg import Audio, GameState
from soccer_vision_2d_msgs.msg import BallArray, GoalpostArray, RobotArray, Robot
from bitbots_vision.vision_modules import debug, ros_utils, candidate

from . import detectors, object_manager, yoeo_handlers

logger = rclpy.logging.get_logger('yoeo_vision_components')


class DebugImageFactory:
    """
    Factory class that handles the lifetime of debug.DebugImage objects used in almost all YOEO vision components.
    """
    _config: Dict = {}
    _debug_image: Optional[debug.DebugImage] = None

    @classmethod
    def get(cls, config: Dict) -> debug.DebugImage:
        """
        Get an instance of debug.DebugImage.
        """
        if cls._new_debug_image_has_to_be_created(config):
            cls._create_new_debug_image(config)
        return cls._debug_image

    @classmethod
    def _new_debug_image_has_to_be_created(cls, config: Dict) -> bool:
        return cls._debug_image is None \
            or ros_utils.config_param_change(cls._config, config, 'component_debug_image_active')

    @classmethod
    def _create_new_debug_image(cls, config: Dict) -> None:
        cls._debug_image = debug.DebugImage(config['component_debug_image_active'])
        cls._config = config


class IVisionComponent(ABC):
    """
    Interface for the YOEO vision components.
    """

    @abstractmethod
    def configure(self, config: Dict, debug_mode: bool) -> None:
        """
        This method is to be used to (re-) configure the vision component. It should be invoked once before running the
        component for the first time.
        """
        ...

    @abstractmethod
    def run(self, image_msg: Image) -> None:
        """
        This method runs the vision component.
        """
        ...

    @abstractmethod
    def set_image(self, image: np.ndarray) -> None:
        """
        This method is to be used to notify a component about a new image.
        """
        ...


class YOEOComponent(IVisionComponent):
    """
    This component runs the YOEO network. It is MANDATORY and must be run BEFORE any of the other components.
    """

    def __init__(self):
        self._yoeo_instance: Optional[yoeo_handlers.IYOEOHandler] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._yoeo_instance = object_manager.YOEOObjectManager.get()

    def run(self, image_msg: Image) -> None:
        self._yoeo_instance.predict()

    def set_image(self, image: np.ndarray) -> None:
        self._yoeo_instance.set_image(image)


class CameraCapCheckComponent(IVisionComponent):
    """
    This component checks if the camera cap could still be attached to the camera.
    Component deactivates itself after the first image.
    """

    def __init__(self, node: Node):
        self._camera_cap_brightness_threshold: int = 0
        self._config: Dict = {}
        self._image: Optional[np.ndarray] = None
        self._is_first_image: bool = True
        self._node: Node = node
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._camera_cap_brightness_threshold = config['vision_blind_threshold']

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config,
            new_config,
            self._publisher,
            'ROS_audio_msg_topic',
            Audio
        )

    def run(self, image_msg: Image) -> None:
        if self._component_has_not_run_yet():
            self._set_component_has_run()
            self._run_component()

    def _component_has_not_run_yet(self) -> bool:
        return self._is_first_image

    def _set_component_has_run(self) -> None:
        self._is_first_image = False

    def _run_component(self) -> None:
        if self._camera_cap_could_be_on():
            self._log_error()
            self._issue_oral_warning()

    def _camera_cap_could_be_on(self) -> bool:
        mean_image_brightness = self._image.mean()
        return mean_image_brightness < self._camera_cap_brightness_threshold

    @staticmethod
    def _log_error() -> None:
        logger.error("Image is too dark! Camera cap not removed?")

    def _issue_oral_warning(self) -> None:
        ros_utils.speak("Hey!   Remove my camera cap!", self._publisher)

    def set_image(self, image: np.ndarray) -> None:
        if self._component_has_not_run_yet():
            self._image = image
        else:
            self._image = None  # to allow for garbage collection of first image


class BallDetectionComponent(IVisionComponent):
    """
    This component carries out the ball detection using YOEO.
    """

    def __init__(self, node: Node):
        self._config: Dict = {}
        self._ball_detector: Optional[detectors.BallDetector] = None
        self._debug_image: Optional[debug.DebugImage] = None
        self._debug_mode: bool = False
        self._node: Node = node
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._ball_detector = detectors.BallDetector(object_manager.YOEOObjectManager.get())
        self._debug_image = DebugImageFactory.get(config)
        self._debug_mode = debug_mode

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config,
            new_config,
            self._publisher,
            'ROS_ball_msg_topic',
            BallArray
        )

    def run(self, image_msg: Image) -> None:
        candidates = self._get_best_ball_candidates()
        final_candidates = self._filter_by_candidate_threshold(candidates)
        self._publish_balls_message(image_msg, final_candidates)

        if self._debug_mode:
            self._add_candidates_to_debug_image(candidates)
            self._add_candidates_within_convex_fb_to_debug_image(candidates)
            self._add_final_candidates_to_debug_image(final_candidates)

    def _get_best_ball_candidates(self) -> List[candidate.Candidate]:
        return self._ball_detector.get_top_candidates(count=self._config['ball_candidate_max_count'])

    def _filter_by_candidate_threshold(self, candidates: List[candidate.Candidate]) \
            -> List[candidate.Candidate]:
        return candidate.Candidate.rating_threshold(candidates, self._config['ball_candidate_rating_threshold'])

    def _publish_balls_message(self, image_msg: Image, candidates: List[candidate.Candidate]) -> None:
        ball_messages = list(map(ros_utils.build_ball_msg, candidates))
        balls_message = ros_utils.build_ball_array_msg(image_msg.header, ball_messages)
        self._publisher.publish(balls_message)

    def _add_candidates_to_debug_image(self, candidates: List[candidate.Candidate]) -> None:
        self._debug_image.draw_ball_candidates(candidates, DebugImageComponent.Colors.ball, thickness=1)

    def _add_candidates_within_convex_fb_to_debug_image(self, candidates: List[candidate.Candidate]) -> None:
        self._debug_image.draw_ball_candidates(candidates, DebugImageComponent.Colors.ball, thickness=2)

    def _add_final_candidates_to_debug_image(self, candidates: List[candidate.Candidate]) -> None:
        self._debug_image.draw_ball_candidates(candidates, DebugImageComponent.Colors.ball, thickness=3)

    def set_image(self, image: np.ndarray) -> None:
        pass


class GoalpostDetectionComponent(IVisionComponent):
    """
    This component carries out the goalpost detection using YOEO.
    """

    def __init__(self, node: Node):
        self._config: Dict = {}
        self._debug_image: Optional[debug.DebugImage] = None
        self._debug_mode: bool = False
        self._goalpost_detector: Optional[detectors.GoalpostDetector] = None
        self._node: Node = node
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._debug_image = DebugImageFactory.get(config)
        self._debug_mode = debug_mode
        self._goalpost_detector = detectors.GoalpostDetector(object_manager.YOEOObjectManager.get())

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config,
            new_config,
            self._publisher,
            'ROS_goal_posts_msg_topic',
            GoalpostArray
        )

    def run(self, image_msg: Image) -> None:
        candidates = self._get_candidates()
        self._publish_goalposts_message(image_msg, candidates)

        if self._debug_mode:
            self._add_candidates_to_debug_image(self._goalpost_detector.get_candidates())
            self._add_final_candidates_to_debug_image(candidates)

    def _get_candidates(self) -> List[candidate.Candidate]:
        return self._goalpost_detector.get_candidates()

    def _publish_goalposts_message(self, image_msg: Image, candidates: List[candidate.Candidate]) -> None:
        goalpost_messages = [ros_utils.build_goal_post_msg(candidate) for candidate in candidates]
        goalposts_message = ros_utils.build_goal_post_array_msg(image_msg.header, goalpost_messages)
        self._publisher.publish(goalposts_message)

    def _add_candidates_to_debug_image(self, candidates: List[candidate.Candidate]) -> None:
        self._debug_image.draw_obstacle_candidates(candidates, DebugImageComponent.Colors.goalposts, thickness=1)

    def _add_final_candidates_to_debug_image(self, candidates: List[candidate.Candidate]) -> None:
        self._debug_image.draw_obstacle_candidates(candidates, DebugImageComponent.Colors.goalposts, thickness=3)

    def set_image(self, image: np.ndarray) -> None:
        pass


class LineDetectionComponent(IVisionComponent):
    """
    This component carries out the line detection using YOEO.
    """

    def __init__(self, node: Node):
        self._config: Dict = {}
        self._debug_image: Optional[debug.DebugImage] = None
        self._debug_mode: bool = False
        self._line_detector: Optional[detectors.ISegmentation] = None
        self._node: Node = node
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._debug_image = DebugImageFactory.get(config)
        self._debug_mode = debug_mode
        self._line_detector = detectors.LineSegmentation(object_manager.YOEOObjectManager.get())

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config,
            new_config,
            self._publisher,
            'ROS_line_mask_msg_topic',
            Image
        )

    def run(self, image_msg: Image) -> None:
        line_mask = self._line_detector.get_mask_image()
        self._publish_line_mask_msg(image_msg, line_mask)

        if self._debug_mode:
            self._add_line_mask_to_debug_image(line_mask)

    def _publish_line_mask_msg(self, image_msg: Image, line_mask: np.ndarray) -> None:
        line_mask_msg = ros_utils.build_image_msg(image_msg.header, line_mask, '8UC1')
        self._publisher.publish(line_mask_msg)

    def _add_line_mask_to_debug_image(self, line_mask: np.ndarray) -> None:
        self._debug_image.draw_mask(line_mask, color=DebugImageComponent.Colors.lines)

    def set_image(self, image: np.ndarray) -> None:
        pass  # Nothing should happen here


class FieldDetectionComponent(IVisionComponent):
    """
    This component carries out the field detection using YOEO.
    """

    def __init__(self, node: Node):
        self._config: Dict = {}
        self._field_detector: Optional[detectors.ISegmentation] = None
        self._node: Node = node
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._field_detector = detectors.FieldSegmentation(object_manager.YOEOObjectManager.get())
        logger.info('Field mask WILL BE published')

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config,
            new_config,
            self._publisher,
            'ROS_field_mask_image_msg_topic',
            Image
        )

    def run(self, image_msg: Image) -> None:
        field_mask = self._get_field_mask()
        self._publish_field_mask_msg(image_msg, field_mask)

    def _get_field_mask(self) -> np.ndarray:
        return self._field_detector.get_mask_image()

    def _publish_field_mask_msg(self, image_msg: Image, field_mask: np.ndarray) -> None:
        field_mask_msg = ros_utils.build_image_msg(image_msg.header, field_mask, '8UC1')
        self._publisher.publish(field_mask_msg)

    def set_image(self, image: np.ndarray) -> None:
        pass  # Nothing should happen here


class RobotDetectionComponent(IVisionComponent):
    """
    This component carries out the robot detection using YOEO with team color detection done by YOEO.
    """

    def __init__(self, node: Node):
        self._config: Dict = {}
        self._debug_image: Optional[debug.DebugImage] = None
        self._debug_mode: bool = False

        self._unknown_detector: Optional[candidate.CandidateFinder] = None
        self._opponents_detector: Optional[candidate.CandidateFinder] = None
        self._team_mates_detector: Optional[candidate.CandidateFinder] = None

        self._node: Node = node
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        own_color, opponent_color = self._determine_team_colors()

        self._configure_detectors(config, own_color, opponent_color)

        self._debug_image = DebugImageFactory.get(config)
        self._debug_mode = debug_mode

        self._register_publisher(config)
        self._config = config

    @staticmethod
    def _determine_team_colors() -> Tuple[Optional[int], Optional[int]]:
        own_color = ros_utils.get_robot_color_for_team(Robot().attributes.TEAM_OWN)
        opponent_color = ros_utils.get_robot_color_for_team(Robot().attributes.TEAM_OPPONENT)
        return own_color, opponent_color

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config,
            new_config,
            self._publisher,
            'ROS_obstacle_msg_topic',
            RobotArray
        )

    def run(self, image_msg: Image) -> None:
        obstacle_msgs: List[Robot] = []
        self._add_team_mates_to(obstacle_msgs)
        self._add_opponents_to(obstacle_msgs)
        self._add_remaining_obstacles_to(obstacle_msgs)

        self._publish_obstacles_message(image_msg, obstacle_msgs)

        if self._debug_mode:
            self._add_obstacles_to_debug_image()

    def _add_team_mates_to(self, obstacle_msgs: List[Robot]) -> None:
        team_mate_candidates = self._team_mates_detector.get_candidates()
        team_mate_candidate_messages = self._create_obstacle_messages(Robot().attributes.TEAM_OWN, team_mate_candidates)
        obstacle_msgs.extend(team_mate_candidate_messages)

    @staticmethod
    def _create_obstacle_messages(obstacle_type: Robot, candidates: List[candidate.Candidate]) -> List[Robot]:
        return [ros_utils.build_robot_msg(obstacle_candidate, obstacle_type) for obstacle_candidate in candidates]

    def _add_opponents_to(self, obstacle_msgs: List[Robot]) -> None:
        opponent_candidates = self._opponents_detector.get_candidates()
        opponent_candidate_messages = self._create_obstacle_messages(
            Robot().attributes.TEAM_OPPONENT,
            opponent_candidates
        )
        obstacle_msgs.extend(opponent_candidate_messages)

    def _add_remaining_obstacles_to(self, obstacle_msgs: List[Robot]) -> None:
        remaining_candidates = self._unknown_detector.get_candidates()
        remaining_candidate_messages = self._create_obstacle_messages(
            Robot().attributes.TEAM_UNKNOWN,
            remaining_candidates
        )
        obstacle_msgs.extend(remaining_candidate_messages)

    def _publish_obstacles_message(self, image_msg: Image, obstacle_msgs: List[Robot]) -> None:
        obstacles_msg = ros_utils.build_robot_array_msg(image_msg.header, obstacle_msgs)
        self._publisher.publish(obstacles_msg)

    def _add_obstacles_to_debug_image(self) -> None:
        self._add_team_mates_to_debug_image()
        self._add_opponents_to_debug_image()
        self._add_remaining_objects_to_debug_image()

    def _add_team_mates_to_debug_image(self) -> None:
        team_mate_candidates = self._team_mates_detector.get_candidates()
        self._debug_image.draw_obstacle_candidates(
            team_mate_candidates,
            DebugImageComponent.Colors.team_mates,
            thickness=3
        )

    def _add_opponents_to_debug_image(self) -> None:
        opponent_candidates = self._opponents_detector.get_candidates()
        self._debug_image.draw_obstacle_candidates(
            opponent_candidates,
            DebugImageComponent.Colors.opponents,
            thickness=3
        )

    def _add_remaining_objects_to_debug_image(self) -> None:
        remaining_candidates = self._unknown_detector.get_candidates()
        self._debug_image.draw_obstacle_candidates(
            remaining_candidates,
            DebugImageComponent.Colors.unknown_obstacles,
            thickness=3
        )

    def set_image(self, image: np.ndarray) -> None:
        self._team_mates_detector.set_image(image)
        self._opponents_detector.set_image(image)
        self._unknown_detector.set_image(image)

    def _configure_detectors(self, config: Dict, own_color: int, opponent_color: int) -> None:
        self._team_mates_detector = self._select_detector_based_on(own_color)
        self._opponents_detector = self._select_detector_based_on(opponent_color)
        self._unknown_detector = self._select_detector_based_on(None)

    @classmethod
    def _select_detector_based_on(cls, team_color: Optional[int]):
        if team_color == GameState.BLUE:
            color_detector = detectors.BlueRobotDetector(object_manager.YOEOObjectManager.get())
        elif team_color == GameState.RED:
            color_detector = detectors.RedRobotDetector(object_manager.YOEOObjectManager.get())
        else:
            color_detector = detectors.UnknownRobotDetector(object_manager.YOEOObjectManager.get())
        return color_detector


class NoTeamColorRobotDetectionComponent(IVisionComponent):
    """
    This component carries out the robot detection using YOEO, albeit not detecting any team colors. Instead, all
    robots are mapped to the "unknown" class.
    """

    def __init__(self, node: Node):
        super().__init__(node)

    def __init__(self, node: Node):
        self._config: Dict = {}
        self._debug_image: Optional[debug.DebugImage] = None
        self._debug_mode: bool = False

        self._robot_detector: Optional[candidate.CandidateFinder] = None

        self._node: Node = node
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._debug_image = DebugImageFactory.get(config)
        self._debug_mode = debug_mode

        self._robot_detector = detectors.RobotDetector(object_manager.YOEOObjectManager.get())

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config,
            new_config,
            self._publisher,
            'ROS_obstacle_msg_topic',
            RobotArray
        )

    def run(self, image_msg: Image) -> None:
        obstacle_msgs: List[Robot] = []
        self._add_robot_obstacles_to(obstacle_msgs)

        self._publish_obstacles_message(image_msg, obstacle_msgs)

        if self._debug_mode:
            self._add_robot_objects_to_debug_image()

    @staticmethod
    def _create_obstacle_messages(obstacle_type: Robot, candidates: List[candidate.Candidate]) -> List[Robot]:
        return [ros_utils.build_robot_msg(obstacle_candidate, obstacle_type) for obstacle_candidate in candidates]

    def _add_robot_obstacles_to(self, obstacle_msgs: List[Robot]) -> None:
        robot_candidates = self._robot_detector.get_candidates()
        robot_candidate_messages = self._create_obstacle_messages(
            Robot().attributes.TEAM_UNKNOWN,
            robot_candidates
        )
        obstacle_msgs.extend(robot_candidate_messages)

    def _publish_obstacles_message(self, image_msg: Image, obstacle_msgs: List[Robot]) -> None:
        obstacles_msg = ros_utils.build_robot_array_msg(image_msg.header, obstacle_msgs)
        self._publisher.publish(obstacles_msg)

    def _add_robot_objects_to_debug_image(self) -> None:
        robot_candidates = self._robot_detector.get_candidates()
        self._debug_image.draw_obstacle_candidates(
            robot_candidates,
            DebugImageComponent.Colors.unknown_obstacles,
            thickness=3
        )

    def set_image(self, image: np.ndarray) -> None:
        self._robot_detector.set_image(image)


class DebugImageComponent(IVisionComponent):
    """
    This component handles and publishes the debug image.
    """

    class Colors:
        # BGR
        ball = (0, 255, 0)  # green
        team_mates = (255, 0, 0)  # blue
        opponents = (0, 0, 255)  # red
        unknown_obstacles = (160, 160, 160)  # grey
        goalposts = (255, 255, 255)  # white
        lines = (255, 0, 0)  # blue

    def __init__(self, node):
        self._config: Dict = {}
        self._node: Node = node
        self._debug_image: Optional[debug.DebugImage] = None
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._debug_image = DebugImageFactory.get(config)
        logger.info('Debug images are published')

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config, config,
            self._publisher,
            'ROS_debug_image_msg_topic',
            Image
        )

    def run(self, image_msg: Image) -> None:
        debug_image_msg = self._create_debug_image_msg(image_msg)
        self._publish_debug_image_msg(debug_image_msg)

    def _create_debug_image_msg(self, image_msg: Image) -> Image:
        return ros_utils.build_image_msg(image_msg.header, self._debug_image.get_image(), 'bgr8')

    def _publish_debug_image_msg(self, debug_image_msg: Image) -> None:
        self._publisher.publish(debug_image_msg)

    def set_image(self, image: np.ndarray) -> None:
        self._debug_image.set_image(image)

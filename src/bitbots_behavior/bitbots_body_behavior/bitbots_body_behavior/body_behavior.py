import importlib.util
import os
import threading
import traceback
from pathlib import Path
from typing import Callable

import rclpy
from bitbots_blackboard.body_blackboard import BodyBlackboard
from bitbots_tf_buffer import Buffer
from builtin_interfaces.msg import Time as TimeMsg
from dynamic_stack_decider.dsd import DSD
from game_controller_hsl_interfaces.msg import GameState
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from soccer_vision_3d_msgs.msg import RobotArray

from bitbots_body_behavior import behavior_dsd
from bitbots_body_behavior import simple_behavior
from bitbots_msgs.msg import RobotControlState, TeamData


def create_blackboard_with_subscribers(node: Node) -> BodyBlackboard:
    tf_buffer = Buffer(Duration(seconds=30), node)
    blackboard = BodyBlackboard(node, tf_buffer)

    node.create_subscription(
        PoseWithCovarianceStamped,
        "ball_position_relative_filtered",
        blackboard.world_model.ball_filtered_callback,
        qos_profile=1,
        callback_group=MutuallyExclusiveCallbackGroup(),
    )
    node.create_subscription(
        GameState,
        "gamestate",
        blackboard.gamestate.gamestate_callback,
        qos_profile=1,
        callback_group=MutuallyExclusiveCallbackGroup(),
    )
    node.create_subscription(
        TeamData,
        "team_data",
        blackboard.team_data.team_data_callback,
        qos_profile=10,
        callback_group=MutuallyExclusiveCallbackGroup(),
    )
    node.create_subscription(
        RobotArray,
        "robots_relative_filtered",
        blackboard.costmap.robot_callback,
        qos_profile=1,
        callback_group=MutuallyExclusiveCallbackGroup(),
    )
    node.create_subscription(
        RobotArray,
        "robots_relative_filtered",
        blackboard.world_model.robot_callback,
        qos_profile=1,
        callback_group=MutuallyExclusiveCallbackGroup(),
    )
    node.create_subscription(
        RobotControlState,
        "robot_state",
        blackboard.misc.robot_state_callback,
        qos_profile=1,
        callback_group=MutuallyExclusiveCallbackGroup(),
    )
    node.create_subscription(
        Twist,
        "cmd_vel",
        blackboard.pathfinding.cmd_vel_cb,
        qos_profile=1,
        callback_group=MutuallyExclusiveCallbackGroup(),
    )
    node.create_subscription(
        TimeMsg,
        "whistle_detected",
        blackboard.gamestate.whistle_detection_callback,
        qos_profile=1,
        callback_group=MutuallyExclusiveCallbackGroup(),
    )

    return blackboard


class BodyBehaviorBase:
    def __init__(self, node: Node):
        self.counter = 0
        self.node = node
        self.blackboard = create_blackboard_with_subscribers(node)

    def _run_behavior_step(self) -> None:
        raise NotImplementedError

    def shutdown(self) -> None:
        pass

    def loop(self):
        try:
            self._run_behavior_step()
            self.blackboard.team_data.publish_strategy()
            self.blackboard.team_data.publish_time_to_ball()
            self.counter = (self.counter + 1) % self.blackboard.config["time_to_ball_divider"]
            if self.counter == 0:
                self.blackboard.pathfinding.calculate_time_to_ball()
            self.blackboard.clear_cache()
        except Exception as e:
            traceback.print_exc()
            self.node.get_logger().error(str(e))


class BodyDSD(BodyBehaviorBase):
    def __init__(self, node: Node):
        super().__init__(node)
        self.dsd = DSD(self.blackboard, "debug/dsd/body_behavior", node)  # TODO: use config

        self.dsd.register_actions(behavior_dsd.actions.__path__[0])
        self.dsd.register_decisions(behavior_dsd.decisions.__path__[0])

        dsd_file: str = node.get_parameter("dsd_file").value
        self.dsd.load_behavior(os.path.join(behavior_dsd.__path__[0], dsd_file))

    def _run_behavior_step(self) -> None:
        self.dsd.update()


class BodySimpleBehavior(BodyBehaviorBase):
    _THREAD_JOIN_TIMEOUT_SECONDS = 0.5

    def __init__(self, node: Node, load_script: bool = True):
        super().__init__(node)
        self._main_function: Callable[[], None] | None = None
        self._main_thread: threading.Thread | None = None
        self._state_lock = threading.Lock()
        self._thread_state = "not_started"
        self._thread_failure_traceback: str | None = None
        self._thread_failure_supervised = False
        simple_behavior.configure_runtime(simple_behavior.SimpleBehaviorRuntime(node, self.blackboard))
        if load_script:
            self._load_script()

    def get_thread_state(self) -> str:
        with self._state_lock:
            return self._thread_state

    def submit_script(self, script_source: str, source_name: str = "simple_behavior_submitted.py") -> tuple[bool, str]:
        if not script_source.strip():
            return False, "Submitted script is empty."

        try:
            compiled_script = compile(script_source, source_name, "exec")
            module_globals: dict[str, object] = {"__name__": "bitbots_user_simple_behavior"}
            exec(compiled_script, module_globals)  # noqa: S102 - required to run user-provided behavior script
        except Exception:
            return False, traceback.format_exc()

        main_function = module_globals.get("main")
        if main_function is None or not callable(main_function):
            return False, "Simple behavior script must provide a callable main() function."

        with self._state_lock:
            if self._thread_state == "running":
                return (
                    False,
                    "Simple behavior main() is already running. Restart body behavior to run a different script.",
                )
            self._main_function = main_function
            self._main_thread = None
            self._thread_state = "not_started"
            self._thread_failure_traceback = None
            self._thread_failure_supervised = False

        return True, "Script accepted."

    def _run_user_main(self) -> None:
        main_function = self._main_function
        if main_function is None:
            return

        try:
            main_function()
        except Exception:
            error_traceback = traceback.format_exc()
            with self._state_lock:
                self._thread_state = "failed"
                self._thread_failure_traceback = error_traceback
            return

        with self._state_lock:
            self._thread_state = "finished"

    def _load_script(self) -> None:
        script_path = str(self.node.get_parameter("simple_behavior_script").value)
        if not script_path:
            self.node.get_logger().error("No simple behavior script configured. Set parameter 'simple_behavior_script'.")
            return

        resolved_script_path = Path(script_path).expanduser()
        if not resolved_script_path.is_absolute():
            resolved_script_path = (Path.cwd() / resolved_script_path).resolve()

        if not resolved_script_path.is_file():
            self.node.get_logger().error(f"Simple behavior script not found: {resolved_script_path}")
            return

        module_spec = importlib.util.spec_from_file_location("bitbots_user_simple_behavior", resolved_script_path)
        if module_spec is None or module_spec.loader is None:
            self.node.get_logger().error(f"Could not load simple behavior script: {resolved_script_path}")
            return

        module = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)

        main_function = getattr(module, "main", None)
        if main_function is None or not callable(main_function):
            self.node.get_logger().error("Simple behavior script must provide a callable main() function.")
            return

        with self._state_lock:
            self._main_function = main_function
            self._main_thread = None
            self._thread_state = "not_started"
            self._thread_failure_traceback = None
            self._thread_failure_supervised = False
        self.node.get_logger().info(f"Loaded simple behavior script: {resolved_script_path}")

    def _run_behavior_step(self) -> None:
        if self._main_function is None:
            return

        should_start = False
        should_log_failed = False
        failure_traceback: str | None = None
        with self._state_lock:
            if self._thread_state == "not_started":
                self._thread_state = "running"
                should_start = True
            elif self._thread_state == "failed" and not self._thread_failure_supervised:
                self._thread_failure_supervised = True
                should_log_failed = True
                failure_traceback = self._thread_failure_traceback

        if should_log_failed:
            self.node.get_logger().error("Simple behavior script main() thread failed. It will not be restarted automatically.")
            if failure_traceback:
                self.node.get_logger().error(failure_traceback)
            return

        if not should_start:
            return

        self.node.get_logger().info("Starting simple behavior script main() in background thread.")
        self._main_thread = threading.Thread(target=self._run_user_main, daemon=True, name="simple_behavior_main")
        self._main_thread.start()
        self.node.get_logger().info("Simple behavior script main() thread started.")

    def shutdown(self) -> None:
        with self._state_lock:
            main_thread = self._main_thread
            thread_state = self._thread_state

        if main_thread is None or not main_thread.is_alive():
            if thread_state == "finished":
                self.node.get_logger().info("Simple behavior script main() thread completed.")
            return

        self.node.get_logger().info(
            f"Waiting up to {self._THREAD_JOIN_TIMEOUT_SECONDS:.1f}s for simple behavior script thread to stop."
        )
        main_thread.join(timeout=self._THREAD_JOIN_TIMEOUT_SECONDS)
        if main_thread.is_alive():
            self.node.get_logger().warn("Simple behavior script thread is still running. Continuing shutdown.")
        else:
            self.node.get_logger().info("Simple behavior script main() thread completed.")


def main(args=None):
    import sys
    print(sys.argv, args, file=sys.stderr)
    rclpy.init(args=args)
    node = Node("body_behavior", automatically_declare_parameters_from_overrides=True)

    body_behavior: BodyBehaviorBase
    behavior_mode = str(node.get_parameter("behavior_mode").value).lower()
    node.get_logger().info(f"Behavior mode: {behavior_mode}")
    if behavior_mode == "simple":
        body_behavior = BodySimpleBehavior(node)
    elif behavior_mode == "dsd":
        body_behavior = BodyDSD(node)
    else:
        node.get_logger().warn(f"Unknown behavior_mode '{behavior_mode}'. Falling back to 'dsd'.")
        body_behavior = BodyDSD(node)

    node.create_timer(1 / 60.0, body_behavior.loop, callback_group=MutuallyExclusiveCallbackGroup(), clock=node.get_clock())

    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        body_behavior.shutdown()
        node.destroy_node()

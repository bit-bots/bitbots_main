from dynamic_stack_decider import AbstractActionElement
from rclpy.logging import get_logger

from bitbots_llm_map.plan_executor import PlanExecutor
from bitbots_llm_map.planner_interface import PlannerInterface
from bitbots_llm_map.problem_generator import ProblemGenerator


class LLMMapFreeKickAction(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

        # Load modules
        self.problem_generator = ProblemGenerator()
        self.planner = PlannerInterface()
        self.executor = PlanExecutor(blackboard, dsd)

        # Blackboard extension
        if not hasattr(self.blackboard, "llm_map"):
            self.blackboard.llm_map = type("obj", (object,), {})()
        self.blackboard.llm_map.plan_success = False
        self.blackboard.llm_map.current_plan = None

        # Logger für ROS 2
        self.logger = get_logger("llm_map_freekick")

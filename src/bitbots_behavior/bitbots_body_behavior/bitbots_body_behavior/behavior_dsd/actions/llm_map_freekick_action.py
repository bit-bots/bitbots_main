from bitbots_blackboard.body_blackboard import BodyBlackboard
from bitbots_llm_map.plan_executor import PlanExecutor
from bitbots_llm_map.planner_interface import PlannerInterface
from bitbots_llm_map.problem_generator import ProblemGenerator
from dynamic_stack_decider import AbstractActionElement
from rclpy.logging import get_logger


class LLMMapFreeKickAction(AbstractActionElement):
    blackboard: BodyBlackboard

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

    def perform(self):
        self.logger.info("[LLM+MAP] FreeKick Action starting – start Planning...")

        # Reading world model
        state = {}
        wm = self.blackboard.world_model

        try:
            state["own_position"] = wm.get_current_position()
            state["ball_position"] = wm.get_ball_position_xy()
            state["ball_seen"] = True  # maybe not always true
            state["free_kick"] = True

            self.logger.info(
                f"Reading WorldModel finished → own: {state['own_position']} | ball: {state['ball_position']}"
            )

        except Exception as e:
            self.logger.error(f"Error in World Model!: {e}")
            # Using minimal fallback
            state = {"own_position": (0.0, 0.0), "ball_position": (2.0, 0.5), "ball_seen": True, "free_kick": True}
            self.logger.warn("→ Fallback state was used!")

        # LLM creates a problem.pddl
        problem_pddl = self.problem_generator.generate(state, task="free_kick")

        # Planner finds a solution for the problem
        result = self.planner.solve(domain=self.blackboard.llm_map.domain_pddl, problem=problem_pddl, timeout=8)

        if result.success:
            self.logger.info(f"[LLM+MAP] plan was found! ({len(result.plan)} steps, {result.time_taken:.2f}s)")
            self.dsd.set_action("ExecuteLLMPlan")
        else:
            self.logger.warn("[LLM+MAP] hasn't found a plan → Fallback")

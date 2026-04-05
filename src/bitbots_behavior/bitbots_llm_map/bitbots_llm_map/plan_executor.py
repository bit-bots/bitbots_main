from rclpy.logging import get_logger


class PlanExecutor:
    def __init__(self, blackboard, dsd):
        self.blackboard = blackboard
        self.dsd = dsd
        self.logger = get_logger("plan_executor")

        self.action_mapping = {
            "walk_to": self._execute_walk_to,
            "position_for_free_kick": self._execute_position_for_free_kick,
            "kick_towards_goal": self._execute_kick_towards_goal,
            "wait_seconds": self._execute_wait_seconds,
        }

    def execute(self, plan):
        if not plan:
            self.logger.warn("Plan is empty - nothing to do")
            return

        self.logger.info(f"Start execution of {len(plan)} steps...")

        for i, step in enumerate(plan):
            action_name = step.get("action")
            params = step.get("params", [])

            self.logger.info(f"Step {i + 1}/{len(plan)}: {action_name} {params}")

            if action_name in self.action_mapping:
                success = self.action_mapping[action_name](params)

                if not success:
                    self.logger.error(f"Step {action_name} failed!")
                    return False

            else:
                self.logger.warn(f"Unknown action: {action_name} -> skip")
                continue

        self.logger.info("Whole plan was successful!")
        return True

    # PDDL: walk_to ?r ?p -> GoToAbsolutePositionFieldFraction
    def _execute_walk_to(self, params):
        if len(params) < 2:
            self.logger("Not enough params")
            return False

        target_symbol = params[1]

        wm = self.blackboard.world_model

        if target_symbol == "pos_ball":
            x = wm.get_ball_position_xy()[0]
            y = wm.get_ball_position_xy()[1]
        elif target_symbol == "pos_goal":
            x = 1.0
            y = 0.0
        else:
            # Fallback
            self.logger.warn(f"Unknown position: {target_symbol}")
            x, y = 0.5, 0.5

        self.logger.info(f"→ Go to position: {target_symbol} → x={x:.2f}, y={y:.2f}")

    # PDDL: position for free kick ?r
    def _execute_position_for_free_kick(self, params):
        if len(params) < 1:
            self.logger.error("position_for_free_kick: no robot mentioned")
            return False

        self.logger.info("Position for a free kick")

        self.dsd.set_action("GoToRolePosition", blocking=False)
        return True

    # PDDL: kick_towards_goal ?r ?target
    def _execute_kick_towards_goal(self, params):
        if len(params) < 2:
            self.logger.error("kick_towards_goal: not enough parameter")
            return False

        target = params[1]

        self.logger.info(f"→ Shoot to goal! (Ziel: {target})")

        self.dsd.set_action("PerformKickLeft", foot="left", r=False)

        return True

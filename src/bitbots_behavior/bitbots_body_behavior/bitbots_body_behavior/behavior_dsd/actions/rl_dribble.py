from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class RLDribble(AbstractActionElement):
    """Runs the learned dribble policy toward the map goal direction.

    While this action is on the stack it keeps the dribble policy active and
    feeds it the kick direction from the map goal (the same smart goal-aimed
    heading the kick uses) together with the configured target ball speed. The
    dribble node takes over the walking motor goals; popping this action stops
    the policy and hands control back to the walk.
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.speed = parameters.get("speed", self.blackboard.config["rl_dribble"]["speed"])

    def perform(self, reevaluate=False):
        # Heading toward the opponent goal from the ball (map frame); the
        # distance only shifts the approach point, not the heading.
        _, _, direction_rad_map, _ = self.blackboard.pathfinding.get_map_goal(distance=0.0)
        self.blackboard.kick.dribble(direction_rad_map, self.speed)

    def on_pop(self):
        self.blackboard.kick.stop_dribble()
        super().on_pop()

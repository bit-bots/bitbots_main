import time
from abc import ABC, abstractmethod

import rclpy
from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from rclpy.task import Future

from bitbots_msgs.action import Dynup


# @TODO: merge/extract with hcm PlayAnimationDynup
class AbstractDynupAnimation(AbstractActionElement, ABC):
    """
    Dynup animation actions are blocking and do not pop themselves!
    This is because otherwise they would, reset themselves directly (e.g. after descend, ascend again)
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.active = False
        self.first_perform = True
        self.dynup_action_current_goal: Future | None = None

    def perform(self, reevaluate=False):
        if self.first_perform:
            # try to start animation
            success = self.start_animation()
            # if we fail, we need to abort this action
            if not success:
                self.blackboard.node.get_logger().error("Could not start animation. Will abort animation action!")
                self.pop()

            self.first_perform = False

    @abstractmethod
    def reset_animation(self):
        """
        This method needs to reset the animation to its initial state.
        For example, if the animation is descend, the robot should stand up again.
        """
        raise NotImplementedError

    def start_animation(self) -> bool:
        """
        This will NOT wait by itself. You have to check animation_finished() by yourself.

        :return: True if the animation was started, False if not
        """
        if not self.is_server_running():
            return False

        # Dynup action server is running, we can start the walkready action
        self.send_animation_goal(self.direction)
        return True

    def send_animation_goal(self, direction: str):
        goal = Dynup.Goal()
        goal.direction = direction
        self.active = True

        self.dynup_action_current_goal = self.blackboard.animation.dynup_action_client.send_goal_async(goal)
        self.dynup_action_current_goal.add_done_callback(self.animation_finished_callback)

    def is_server_running(self) -> bool:
        server_running = self.blackboard.animation.dynup_action_client.wait_for_server(timeout_sec=1)
        if not server_running:
            while not server_running and rclpy.ok():
                self.blackboard.node.get_logger().warn(
                    "Dynup Action Server not running! Dynup cannot work without dynup server! "
                    "Will now wait until server is accessible!",
                    throttle_duration_sec=10.0,
                )
                server_running = self.blackboard.animation.dynup_action_client.wait_for_server(timeout_sec=1)
            if server_running:
                self.blackboard.node.get_logger().warn("Dynup server now running, 'DynupAnimation' action will go on.")
            else:
                self.blackboard.node.get_logger().warn("Dynup server did not start.")
                return False

        return server_running

    def stop_animation(self):
        if self.dynup_action_current_goal is not None:
            self.dynup_action_current_goal.result().cancel_goal_async()

    def on_pop(self):
        """
        Cancel the current goal when the action is popped
        """
        super().on_pop()
        if not self.is_animation_finished():
            self.stop_animation()

        self.set_inactive()
        self.reset_animation()

    def animation_finished_callback(self, animation_done_future: Future):
        """
        Dynup animation future callback, setting the action when the animation is finished
        """
        animation_done_future.result().get_result_async().add_done_callback(lambda _: self.set_inactive())

    def is_animation_finished(self) -> bool:
        return not self.active

    def set_inactive(self):
        self.active = False

    def set_active(self):
        self.active = True


class Descend(AbstractDynupAnimation):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.direction = Dynup.Goal.DIRECTION_DESCEND
        self.reset_direction = Dynup.Goal.DIRECTION_RISE

    def reset_animation(self):
        self.set_active()
        self.send_animation_goal(self.reset_direction)

        while not self.is_animation_finished():
            time.sleep(0.1)


class GetWalkready(AbstractDynupAnimation):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.direction = Dynup.Goal.DIRECTION_WALKREADY

    def reset_animation(self):
        pass

    def perform(self, reevaluate=False):
        super().perform(reevaluate)

        if self.is_animation_finished():
            self.pop()

import rclpy
from bitbots_msgs.action import Dynup
from dynamic_stack_decider.abstract_action_element import AbstractActionElement

from bitbots_blackboard.blackboard import BodyBlackboard


class GetWalkready(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.direction = "walkready"
        self.first_perform = True
        self.active = False

    def perform(self, reevaluate=False):
        # deactivate falling since it will be wrongly detected
        self.do_not_reevaluate()
        if self.first_perform:
            # get the animation that should be played
            # defined by implementations of this abstract class

            # try to start animation
            success = self.start_animation()
            # if we fail, we need to abort this action
            if not success:
                self.blackboard.node.get_logger().error("Could not start animation. Will abort play animation action!")
                return self.pop()

            self.first_perform = False
            return

        if self.animation_finished():
            # we are finished playing this animation
            return self.pop()

    def start_animation(self) -> bool:
        """
        This will NOT wait by itself. You have to check
        animation_finished() by yourself.

        :return: True if the animation was started, False if not
        """
        server_running = self.blackboard.animation.dynup_action_client.wait_for_server(timeout_sec=1.0)
        if not server_running:
            while not server_running and rclpy.ok():
                self.blackboard.node.get_logger().warn(
                    "Dynup Action Server not running! Dynup cannot work without dynup server! "
                    "Will now wait until server is accessible!",
                    throttle_duration_sec=10.0,
                )
                server_running = self.blackboard.animation.dynup_action_client.wait_for_server(timeout_sec=1)
            if server_running:
                self.blackboard.node.get_logger().warn("Dynup server now running, 'get_walkready' action will go on.")
            else:
                self.blackboard.node.get_logger().warn("Dynup server did not start.")
                return False

        # Dynup action server is running, we can start the walkready action
        goal = Dynup.Goal()
        goal.direction = self.direction
        self.active = True
        self.dynup_action_current_goal = self.blackboard.animation.dynup_action_client.send_goal_async(goal)
        self.dynup_action_current_goal.add_done_callback(
            lambda future: future.result()
            .get_result_async()
            .add_done_callback(lambda result_future: self.__done_cb(result_future))
        )
        return True

    def __done_cb(self, result_future):
        self.active = False

    def animation_finished(self) -> bool:
        """
        Checks if the animation is finished.

        :return: True if the animation is finished, False if not
        """
        return not self.active

import rclpy
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from bitbots_msgs.action import Dynup
from action_msgs.msg import GoalStatus

class GetWalkready(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters=None)
        self.direction = 'walkready'
        self.first_perform = True

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

    def start_animation(self):
        """
        This will NOT wait by itself. You have to check
        animation_finished()
        by yourself.
        :return:
        """

        first_try = self.blackboard.dynup_action_client.wait_for_server(timeout_sec=1.0)
        if not first_try:
            server_running = False
            while not server_running and rclpy.ok():
                self.blackboard.node.get_logger().warn(
                                      "Dynup Action Server not running! Dynup cannot work without dynup server!"
                                      "Will now wait until server is accessible!",
                                      throttle_duration_sec=10.0)
                server_running = self.blackboard.dynup_action_client.wait_for_server(timeout_sec=1)
            if server_running:
                self.blackboard.node.get_logger().warn("Dynup server now running, hcm will go on.")
            else:
                self.blackboard.node.get_logger().warn("Dynup server did not start.")
                return False
        goal = Dynup.Goal()
        goal.direction = self.direction
        self.dynup_action_current_goal = self.blackboard.dynup_action_client.send_goal_async(goal)
        return True

    def animation_finished(self):
        return (self.dynup_action_current_goal.done() and self.dynup_action_current_goal.result().status == GoalStatus.STATUS_SUCCEEDED) \
                or self.dynup_action_current_goal.cancelled()

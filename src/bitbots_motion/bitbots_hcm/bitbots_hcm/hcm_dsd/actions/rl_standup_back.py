"""HCM action element that triggers the RL standup-back policy.

Modeled on ``PlayAnimationDynup`` from ``play_animation.py``: the action
holds a goal open on a ROS action server while the standup policy runs and
publishes joint commands. When the DSD pops this action (e.g. because the
robot is no longer detected as fallen), the goal is canceled and the policy
node stops publishing.

Notes
-----
* Reuses the existing ``bitbots_msgs/Dynup`` action type, with
  ``direction = "rl_standup_back"``. No new action message is required.
* Creates its own ``ActionClient`` rather than relying on a blackboard field,
  so ``hcm_blackboard.py`` does not need to be modified.
* This file is provided so the action *exists*; it is not yet wired into
  ``hcm.dsd``. Add ``@RLStandupBack`` to the desired DSD branch to use it.
"""

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient

from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement
from bitbots_msgs.action import Dynup

GOAL_DIRECTION = "rl_standup_back"
ACTION_NAME = "rl_standup_back"


class RLStandupBack(AbstractHCMActionElement):
    """DSD action that runs the RL standup-back policy until cancelled."""

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.first_perform = True
        self._action_client = ActionClient(self.blackboard.node, Dynup, ACTION_NAME)
        self._goal_future = None

    def perform(self, reevaluate=False):
        # Don't reevaluate while the policy is running — same pattern as
        # PlayAnimationDynup for FRONT/BACK directions.
        self.do_not_reevaluate()

        if self.first_perform:
            if not self._start_goal():
                self.blackboard.node.get_logger().error(
                    "Could not start RL standup-back goal. Aborting action."
                )
                return self.pop()
            self.first_perform = False
            return

        if self._goal_finished():
            return self.pop()

    def on_pop(self):
        """Cancel the goal if the action is popped before completion."""
        super().on_pop()
        if self._goal_future is None:
            return
        if not self._goal_future.done():
            return
        goal_handle = self._goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return
        if not self._goal_finished():
            goal_handle.cancel_goal_async()

    # ------------------------------------------------------------------ helpers

    def _start_goal(self) -> bool:
        wait_param = self.blackboard.node.get_parameter("hcm.anim_server_wait_time").value
        if not self._action_client.wait_for_server(timeout_sec=wait_param):
            self.blackboard.node.get_logger().warn(
                "RL standup-back action server not running; cannot start goal."
            )
            return False

        goal = Dynup.Goal()
        goal.direction = GOAL_DIRECTION
        goal.from_hcm = True
        self._goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        return True

    def _feedback_cb(self, msg):
        feedback: Dynup.Feedback = msg.feedback
        self.publish_debug_data("RL Standup Back Percent Done", str(feedback.percent_done))

    def _goal_finished(self) -> bool:
        if self._goal_future is None or not self._goal_future.done():
            return False
        if self._goal_future.cancelled():
            return True
        goal_handle = self._goal_future.result()
        if goal_handle is None:
            return False
        return goal_handle.status in (
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        )

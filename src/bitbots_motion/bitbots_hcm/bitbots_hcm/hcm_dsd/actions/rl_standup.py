"""HCM action elements that run an RL standup policy.

Modeled on :class:`AbstractPlayAnimation`: the action holds a goal open on the
standup policy's action server while the policy publishes motor goals, and pops
once the goal finished. If the DSD pops the action earlier (e.g. because the
robot is no longer detected as fallen), the goal is cancelled, which makes the
policy node stop publishing.

The action client comes from the blackboard, like the animation one.
``ActionClient.__init__`` registers its waitable on the node before it assigns
its own lock, so constructing one lazily here — while the HCM node is already
being spun — races the executor's wait set rebuild and crashes the spin thread.

``hcm.dsd`` uses these in the FALLEN_FRONT/FALLEN_BACK branches. The policy node
serving them is started with ``standup:=true`` (and ``mjlab_getup:=false``) in
``rl_motion.launch``; ``direction`` picks the front or back policy inside that
node.
"""

from abc import ABC, abstractmethod

from action_msgs.msg import GoalStatus

from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement
from bitbots_msgs.action import RLStandup

SERVER_WAIT_TIME_SEC = 2.0


class AbstractRLStandup(AbstractHCMActionElement, ABC):
    """Abstract class to create actions running an RL standup policy."""

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.first_perform = True
        self._action_client = self.blackboard.rl_standup_action_client
        self._goal_future = None

    @abstractmethod
    def choose_direction(self) -> str:
        # this is what has to be implemented returning the standup direction
        raise NotImplementedError

    def perform(self, reevaluate=False):
        # we never want to leave the action while the policy is running
        self.do_not_reevaluate()

        if self.first_perform:
            if not self._start_goal(self.choose_direction()):
                self.blackboard.node.get_logger().error("Could not start RL standup. Will abort standup action!")
                return self.pop()

            self.first_perform = False
            return

        if self._goal_finished():
            # the policy is done, either because it succeeded or timed out
            return self.pop()

    def on_pop(self):
        """Cancel a still running goal so the policy stops publishing motor goals."""
        super().on_pop()
        goal_handle = self._accepted_goal_handle()
        if goal_handle is not None and not self._goal_finished():
            goal_handle.cancel_goal_async()

    def _start_goal(self, direction: str) -> bool:
        """Send the standup goal.

        This will NOT wait by itself. You have to check ``_goal_finished()``
        by yourself.
        """
        self.blackboard.node.get_logger().info(f"Starting RL standup '{direction}'")

        if not self._action_client.wait_for_server(timeout_sec=SERVER_WAIT_TIME_SEC):
            self.blackboard.node.get_logger().warn(
                "RL standup action server is not running, so the policy cannot be started."
            )
            return False

        goal = RLStandup.Goal()
        goal.direction = direction
        self._goal_future = self._action_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        self.blackboard.rl_standup_action_current_goal = self._goal_future
        return True

    def _feedback_cb(self, msg):
        feedback: RLStandup.Feedback = msg.feedback
        self.publish_debug_data("RL Standup Percent Done", str(feedback.percent_done))

    def _accepted_goal_handle(self):
        """The handle of the accepted goal, or None while it is not (yet) accepted."""
        if self._goal_future is None or self._goal_future.cancelled() or not self._goal_future.done():
            return None
        goal_handle = self._goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return None
        return goal_handle

    def _goal_finished(self) -> bool:
        if self._goal_future is None:
            return False
        if self._goal_future.cancelled():
            return True
        if not self._goal_future.done():
            # the goal request is still in flight
            return False
        goal_handle = self._accepted_goal_handle()
        if goal_handle is None:
            # the goal was rejected, so nothing is running anymore
            return True
        return goal_handle.status in [
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        ]


class RLStandupBack(AbstractRLStandup):
    def choose_direction(self) -> str:
        self.blackboard.node.get_logger().info("RL STANDUP BACK")
        return RLStandup.Goal.DIRECTION_BACK


class RLStandupFront(AbstractRLStandup):
    def choose_direction(self) -> str:
        self.blackboard.node.get_logger().info("RL STANDUP FRONT")
        return RLStandup.Goal.DIRECTION_FRONT

import rclpy
from action_msgs.msg import GoalStatus

from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement
from bitbots_msgs.action import BeyondMimic


class PerformStandup(AbstractHCMActionElement):
    """
    Triggers a BeyondMimic standup (getup) RL motion via the ``beyondmimic_standup`` action
    server and stays in this action until the motion clip has finished.

    This mirrors :class:`AbstractPlayAnimation` (and the body behavior's ``PerformAcrobatic``):
    the HCM has already set the ``GETTING_UP`` robot state before this action runs (see
    ``hcm.dsd``), so the standup node's motor goals are forwarded on ``getup_motor_goals`` and
    falling detection stays disabled while the policy plays. The ``variant`` DSD parameter
    selects the front (prone) or back (supine) getup; an empty string uses the node's
    configured default variant.

    Usage in a .dsd file:
        @PerformStandup + variant:front
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.variant = parameters.get("variant", "")
        self.first_perform = True

    def perform(self, reevaluate=False):
        # Once the motion is running we never want to be re-evaluated out of it.
        self.do_not_reevaluate()

        if self.first_perform:
            # try to start the motion
            success = self.start_motion()

            # if we fail, abort this action
            if not success:
                self.blackboard.node.get_logger().error(
                    "Could not start BeyondMimic standup motion. Will abort perform standup action!"
                )
                return self.pop()

            self.first_perform = False
            return

        if self.motion_finished():
            # the clip finished (or was cancelled/aborted), so we are done
            return self.pop()

    def start_motion(self) -> bool:
        """
        Sends the goal to the beyondmimic_standup action server. Does NOT wait for the motion
        to finish; check :meth:`motion_finished` for that.
        :return: whether the goal could be sent
        """
        self.blackboard.node.get_logger().info(f"Performing standup motion (variant '{self.variant or '<default>'}')")
        first_try = self.blackboard.beyondmimic_standup_action_client.wait_for_server(
            timeout_sec=self.blackboard.node.get_parameter("hcm.anim_server_wait_time").value
        )
        if not first_try:
            server_running = False
            while not server_running and rclpy.ok():
                self.blackboard.node.get_logger().warn(
                    "beyondmimic_standup action server not running! Standup motion cannot run without it. "
                    "Will now wait until the server is accessible!",
                    throttle_duration_sec=10.0,
                )
                server_running = self.blackboard.beyondmimic_standup_action_client.wait_for_server(timeout_sec=1)
            if server_running:
                self.blackboard.node.get_logger().warn("beyondmimic_standup server now running, hcm will go on.")
            else:
                self.blackboard.node.get_logger().warn("beyondmimic_standup server did not start.")
                return False

        goal = BeyondMimic.Goal()
        goal.motion = self.variant
        self.blackboard.beyondmimic_standup_action_current_goal = (
            self.blackboard.beyondmimic_standup_action_client.send_goal_async(
                goal, feedback_callback=self.motion_feedback_cb
            )
        )
        return True

    def motion_feedback_cb(self, msg):
        feedback: BeyondMimic.Feedback = msg.feedback
        self.publish_debug_data("Standup Progress", str(feedback.progress))

    def motion_finished(self) -> bool:
        assert self.blackboard.beyondmimic_standup_action_current_goal is not None, (
            "No BeyondMimic standup action goal set, so we cannot check if it is finished"
        )
        return (
            self.blackboard.beyondmimic_standup_action_current_goal.done()
            and self.blackboard.beyondmimic_standup_action_current_goal.result().status
            in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]
        ) or self.blackboard.beyondmimic_standup_action_current_goal.cancelled()

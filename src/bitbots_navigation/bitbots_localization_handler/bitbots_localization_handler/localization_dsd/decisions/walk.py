import numpy as np
import tf2_ros as tf2
from rclpy.time import Time

from bitbots_localization_handler.localization_dsd.decisions import AbstractLocalizationDecisionElement


class WalkedSinceLastInit(AbstractLocalizationDecisionElement):
    """
    Decides if we walked significantly since our last initialization
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.distance_threshold = parameters.get("dist", 0.5)

    def perform(self, reevaluate=False):
        if not self.blackboard.use_sim_time:
            # in real life we always have moved and are not teleported
            return "YES"

        if self.blackboard.last_init_odom_transform is None:
            return "YES"  # We don't know the last init state so we say that we moved away from it

        try:
            odom_transform = self.blackboard.tf_buffer.lookup_transform(
                self.blackboard.odom_frame, self.blackboard.base_footprint_frame, Time()
            )
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            self.blackboard.node.get_logger().error(
                f"Reset localization to last init state, because we got up and have no tf: {e}"
            )
            # We assume that we didn't walk if the tf lookup fails
            return "YES"

        walked_distance = np.linalg.norm(
            np.array([odom_transform.transform.translation.x, odom_transform.transform.translation.y])
            - np.array(
                [
                    self.blackboard.last_init_odom_transform.transform.translation.x,
                    self.blackboard.last_init_odom_transform.transform.translation.y,
                ]
            )
        )

        if walked_distance < self.distance_threshold:
            return "NO"
        else:
            return "YES"

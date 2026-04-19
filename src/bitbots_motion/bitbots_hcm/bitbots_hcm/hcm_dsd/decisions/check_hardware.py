from typing import Optional

from rclpy.time import Time

from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement
from bitbots_msgs.msg import RobotControlState


class CheckMotors(AbstractHCMDecisionElement):
    """
    Checks if we are getting information from the motors.
    Since the HCM is not able to work without motor connection, we will stop if there are no values.
    Needs to be checked before other sensors, since they also need the power to be able to response
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.had_problem = False
        self.power_was_off = False

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        # we check if the joint state values are actually changing, since the joint_state controller will publish the same message
        # even if there is no connection anymore. But we don't want to go directly to hardware error if we just
        # have a small break, since this can happen often due to loose cabling
        if (
            self.blackboard.previous_joint_state is not None
            and self.blackboard.current_joint_state is not None
            and (
                self.blackboard.previous_joint_state.effort != self.blackboard.current_joint_state.effort
                or self.blackboard.previous_joint_state.position != self.blackboard.current_joint_state.position
            )
            and not self.blackboard.servo_diag_error
        ):
            self.blackboard.last_different_joint_state_time = self.blackboard.node.get_clock().now()

        if self.blackboard.visualization_active:
            # we will have no problems with hardware in visualization
            return "OKAY"

        if self.blackboard.simulation_active:
            # Some simulators will give the exact same joint messages, which could look like errors,
            # as the real world ros controller will always publish the same message if there is no connection
            # so we will just the check if the message is changing in simulation
            if self.blackboard.current_joint_state is None:
                return "MOTORS_NOT_STARTED"
            else:
                return "OKAY"

        if self.blackboard.servo_overload:
            return "OVERLOAD"
        # We comment this out for now, but maybe add this later again
        # elif self.blackboard.servo_overheat:
        #     return "OVERHEAT"

        # Check if we get no messages or always the exact same
        if (
            self.blackboard.last_different_joint_state_time is None
            or self.blackboard.node.get_clock().now().nanoseconds
            - self.blackboard.last_different_joint_state_time.nanoseconds
            > self.blackboard.motor_timeout_duration * 1e9
        ):
            # Check if the motors have power
            if self.blackboard.is_power_on:
                # If we are currently in startup phase, we will wait a bit before we complain
                if (
                    self.blackboard.current_state == RobotControlState.STARTUP
                    and self.blackboard.node.get_clock().now().nanoseconds - self.blackboard.start_time.nanoseconds
                    < 10 * 1e9
                ):
                    # we are still in startup phase, just wait and dont complain
                    return "MOTORS_NOT_STARTED"
                else:
                    # tell that we have a hardware problem
                    self.had_problem = True
                    # wait for motors to connect
                    return "PROBLEM"
            else:
                # The motors are off, so we will not complain
                self.power_was_off = True
                return "MOTORS_NOT_STARTED"

        elif self.power_was_off:
            # motors are now on and we can continue
            self.blackboard.node.get_logger().info("Motors are now connected. Will resume.")
            self.power_was_off = False
            # But we want to perform a clean start, so we don't jump directly into the last goal position
            return "TURN_ON"

        if self.had_problem:
            # had problem before, just tell that this is solved now
            self.blackboard.node.get_logger().info("Motors are now connected. Will resume.")
            self.had_problem = False

        # motors are on and we can continue
        return "OKAY"

    def get_reevaluate(self):
        return True


class CheckIMU(AbstractHCMDecisionElement):
    """
    Checks if we are getting information from the IMU.
    Since the HCM can not detect falls without it, we will shut everything down if we dont have an imu.
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.had_problem = False

    def perform(self, reevaluate=False):
        if self.blackboard.visualization_active:
            # In visualization, we do not have an IMU. Therefore, return OKAY to ignore that.
            return "OKAY"

        # we will get always the same message if there is no connection, so check if it differs
        if (
            self.blackboard.previous_imu_msg is not None
            and self.blackboard.imu_msg is not None
            and not self.blackboard.previous_imu_msg.orientation == self.blackboard.imu_msg.orientation
            and not self.blackboard.imu_diag_error
        ):
            self.blackboard.last_different_imu_state_time = self.blackboard.node.get_clock().now()

        if self.blackboard.simulation_active:
            # Some simulators will give exact same IMU messages which look like errors, so ignore this case
            if self.blackboard.imu_msg is None:
                return "IMU_NOT_STARTED"
            else:
                return "OKAY"

        if (
            self.blackboard.previous_imu_msg is None
            or self.blackboard.last_different_imu_state_time is None
            or (
                self.blackboard.node.get_clock().now().nanoseconds
                - self.blackboard.last_different_imu_state_time.nanoseconds
                > self.blackboard.imu_timeout_duration * 1e9
            )
        ):
            if (
                self.blackboard.current_state == RobotControlState.STARTUP
                and self.blackboard.node.get_clock().now().nanoseconds - self.blackboard.start_time.nanoseconds
                < 10 * 1e9
            ):
                # wait for the IMU to start
                return "IMU_NOT_STARTED"
            else:
                self.had_problem = True
                return "PROBLEM"

        if self.had_problem:
            # had problem before, just tell that this is solved now
            self.blackboard.node.get_logger().info("IMU is now connected. Will resume.")
            self.had_problem = False

        return "OKAY"

    def get_reevaluate(self):
        return True


class CheckBattery(AbstractHCMDecisionElement):
    """
    Checks the battery level
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.last_battery_high_time: Optional[Time] = None

    def perform(self, reevaluate=False):
        if self.blackboard.battery_voltage is None or self.blackboard.battery_voltage >= self.blackboard.battery_voltage_threshold:
            self.last_battery_high_time = self.blackboard.node.get_clock().now()

        if self.last_battery_high_time is None or (
            self.blackboard.node.get_clock().now().nanoseconds - self.last_battery_high_time.nanoseconds
            > self.blackboard.battery_debounce_time * 1e9
        ):
            return "LOW"

        return "OKAY"

    def get_reevaluate(self):
        return True

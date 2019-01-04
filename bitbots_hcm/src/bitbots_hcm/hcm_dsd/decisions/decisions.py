import rospy
import math
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
import humanoid_league_msgs.msg
from bitbots_hcm.hcm_dsd.hcm_blackboard import STATE_ANIMATION_RUNNING, STATE_CONTROLABLE, STATE_FALLEN, STATE_FALLING, \
    STATE_HARDWARE_PROBLEM, STATE_MOTOR_OFF, STATE_PENALTY, STATE_PICKED_UP, STATE_RECORD, STATE_SHUT_DOWN, \
    STATE_STARTUP, STATE_WALKING, STATE_HCM_OFF


class StartHCM(AbstractDecisionElement):
    """
    Initializes HCM. 
    """

    def perform(self, reevaluate=False):
        if self.blackboard.shut_down_request:
            self.blackboard.current_state = STATE_SHUT_DOWN
            return "SHUTDOWN"
        else:
            if not reevaluate:
                self.blackboard.current_state = STATE_STARTUP
            if self.blackboard.simulation_active:
                return "SIMULATION"
            else:
                return "ROBOT"

    def get_reevaluate(self):
        return True


class CheckIMU(AbstractDecisionElement):
    """
    Checks if we are getting information from the IMU.
    Since the HCM can not detect falls without it, we will shut everything down if we dont have an imu.
    """

    def perform(self, reevaluate=False):
        if not self.blackboard.last_imu_update_time:
            # wait for the IMU to start
            self.blackboard.current_state = STATE_STARTUP
            return "IMU_NOT_STARTED"
        elif self.blackboard.current_time.to_sec() - self.blackboard.last_imu_update_time.to_sec() > self.blackboard.imu_timeout_duration:
            # tell that we have a hardware problem
            self.blackboard.current_state = STATE_HARDWARE_PROBLEM
            return "PROBLEM"
        elif not reevaluate and self.blackboard.current_state == STATE_HARDWARE_PROBLEM:
            # had IMU problem before, just tell that this is solved now
            ROS_INFO("IMU is now connected. Will resume.")
        return "CONNECTION"

    def get_reevaluate(self):
        return True


class CheckPressureSensor(AbstractDecisionElement):
    """
    Checks connection to pressure sensors.
    """

    def perform(self, reevaluate=False):
        if self.blackboard.pressure_sensors_installed and not self.blackboard.simulation_active:
            if not self.blackboard.last_pressure_update_time:
                # wait for the IMU to start
                self.blackboard.current_state = STATE_STARTUP
                return "PRESSURE_NOT_STARTED"
            elif self.blackboard.current_time.to_sec() - self.blackboard.last_pressure_update_time.to_sec() > self.blackboard.pressure_timeout_duration:
                # tell that we have a hardware problem
                self.blackboard.current_state = STATE_HARDWARE_PROBLEM
                return "PROBLEM"
        return "CONNECTION"

    def get_reevaluate(self):
        return True


class Penalty(AbstractDecisionElement):
    """
    Initializes HCM
    """

    def perform(self, reevaluate=False):
        if self.blackboard.penalized:
            # we do an action sequence to go into penalty and to stay there      
            return "PENALIZED"
        else:
            return "FREE"

    def get_reevaluate(self):
        return True


class MotorOffTimer(AbstractDecisionElement):
    """
    Decides on switching servo power
    """

    def perform(self, reevaluate=False):
        if self.blackboard.simulation_active:
            return "SIMULATION"
        # check if the time is reached
        if self.blackboard.current_time.to_sec() - self.blackboard.last_motor_goal_time.to_sec() > self.blackboard.motor_off_time:
            rospy.logwarn_throttle(5, "Didn't recieve goals for " + str(
                self.blackboard.motor_off_time) + " seconds. Will shut down the motors and wait for commands.")
            self.blackboard.current_state = STATE_MOTOR_OFF
            # we do an action sequence to turn off the motors and stay in motor off  
            return "TURN_MOTORS_OFF"
        elif not self.blackboard.current_time.to_sec() - self.blackboard.last_motor_update_time.to_sec() < 0.1:
            # we have to turn the motors on
            return "TURN_MOTORS_ON"
        else:
            # motors are on and we can continue
            return "MOTORS_ARE_ON"

    def get_reevaluate(self):
        return True


class CheckMotors(AbstractDecisionElement):
    """
    Checks if we are getting information from the motors.
    Since the HCM is not able to work without motor connection, we will stop if there are no values.
    """

    def perform(self, reevaluate=False):
        if not self.blackboard.current_time.to_sec() - self.blackboard.last_motor_update_time.to_sec() < 0.1:
            # tell that we have a hardware problem                            
            self.blackboard.current_state = STATE_HARDWARE_PROBLEM
            # wait for motors to connect
            return "PROBLEM"
        return "CONNECTION"

    def get_reevaluate(self):
        return True


class Record(AbstractDecisionElement):
    """
    Decides if the robot is currently recording animations
    """

    def perform(self, reevaluate=False):
        # check if the robot is currently recording animations
        if self.blackboard.record_active:
            self.blackboard.current_state = STATE_RECORD
            return "RECORD_ACTIVE"
        else:
            # robot is not recording
            return "FREE"

    def get_reevaluate(self):
        return True


class PickedUp(AbstractDecisionElement):
    """
    Decides if the robot is currently picked up
    """

    def perform(self, reevaluate=False):
        # check if the robot is currently beeing picked up
        if self.blackboard.pressure_sensors_installed and sum(self.blackboard.pressure) < 1:
            self.blackboard.current_state = STATE_PICKED_UP
            # we do an action sequence to tgo to walkready and stay in picked up state            
            return "PICKED_UP"

        # robot is not picked up
        return "ON_GROUND"

    def get_reevaluate(self):
        return True


class Fallen(AbstractDecisionElement):
    """
    Decides if the robot is fallen and lying on the ground
    """

    def perform(self, reevaluate=False):
        # check if the robot is currently laying on the ground
        fallen_side = self.fall_checker.check_fallen(self.blackboard.smooth_accel, self.blackboard.gyro)

        if self.blackboard.is_stand_up_active and fallen_side is not None:
            self.blackboard.current_state = STATE_FALLEN
            # we play a stand up animation            
            if fallen_side == self.blackboard.fall_checker.FRONT:
                return "FALLEN_FRONT"
            if fallen_side == self.blackboard.fall_checker.BACK:
                return "FALLEN_BACK"
            if fallen_side == self.blackboard.fall_checker.SIDE:
                return "FALLEN_SIDE"
        else:
            # robot is not fallen
            return "NOT_FALLEN"

    def get_reevaluate(self):
        return True


class Falling(AbstractDecisionElement):
    """
    Decides if the robot is currently falling and has to act on this
    """

    def perform(self, reevaluate=False):
        # check if the robot is currently falling
        falling_direction = self.fall_checker.check_falling(self.blackboard.gyro, self.blackboard.quaternion)
        if self.blackboard.falling_detection_active and falling_direction is not None:
            self.blackboard.current_state = STATE_FALLING

            if falling_direction == self.blackboard.fall_checker.FRONT:
                return "FALLING_FRONT"
            if falling_direction == self.blackboard.fall_checker.BACK:
                return "FALLING_BACK"
            if falling_direction == self.blackboard.fall_checker.LEFT:
                return "FALLING_LEFT"
            if falling_direction == self.blackboard.fall_checker.RIGHT:
                return "FALLING_RIGHT"        
        else:
            # robot is not fallen
            return "NOT_FALLING"

    def get_reevaluate(self):
        return True


class BehaviorAnimation(AbstractDecisionElement):
    """
    Decides if the robot is currently wants to play an animation comming from the behavior
    """

    def perform(self, reevaluate=False):
        if self.blackboard.external_animation_running:
            self.blackboard.current_state = STATE_ANIMATION_RUNNING
            return "ANIMATION_RUNNING"
        else:
            return "FREE"

    def get_reevaluate(self):
        return True


class Walking(AbstractDecisionElement):
    """
    Decides if the robot is currently walking
    """

    def perform(self, reevaluate=False):
        if self.blackboard.current_time.to_sec() - self.blackboard.last_walking_goal_time.to_sec() < 0.1:
            self.blackboard.current_state = STATE_WALKING
            if self.blackboard.animation_requested:
                self.blackboard.animation_requested = False
                # we are walking but we have to stop to play an animation
                return "STOP_WALKING"
            else:
                # we are walking and can stay like this
                return "STAY_WALKING"
        else:
            return "NOT_WALKING"

    def get_reevaluate(self):
        return True


class Controlable(AbstractDecisionElement):
    """
    Decides if the robot is currently controlable
    """

    def perform(self, reevaluate=False):
        # check if the robot is in a walkready pose
        if not self.is_walkready():
            self.blackboard.current_state = STATE_ANIMATION_RUNNING
            return "NOT_WALKREADY"
        else:
            self.blackboard.current_state = STATE_CONTROLABLE
            return "WALKREADY"

    def is_walkready(self):
        """
        We check if any joint is has an offset from the walkready pose which is higher than a threshold
        """
        if self.blackboard.current_joint_positions is None:
            return False
        i = 0
        for joint_name in self.blackboard.current_joint_positions.name:
            if joint_name == "HeadPan" or joint_name == "HeadTilt":
                # we dont care about the head position
                i += 1
                continue
            if abs(math.degrees(self.blackboard.current_joint_positions.position[i]) -
                           self.blackboard.walkready_pose_dict[joint_name]) > self.blackboard.walkready_pose_threshold:
                return False
            i += 1
        return True

    def get_reevaluate(self):
        return True

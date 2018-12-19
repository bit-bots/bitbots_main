import rospy
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
        if self.blackboard.last_imu_update_time.to_sec == 0:
            # wait for the IMU to start
            self.blackboard.current_state = STATE_STARTUP
            return "IMU_NOT_STARTED"
        elif self.blackboard.is_imu_timeout():
            # tell that we have a hardware problem
            self.blackboard.current_state = STATE_HARDWARE_PROBLEM
            # wait for IMU
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
        if self.blackboard.is_motor_off_time():
            rospy.logwarn_throttle(5, "Didn't recieve goals for " + str(
                self.blackboard.motor_off_time) + " seconds. Will shut down the motors and wait for commands.")
            self.blackboard.current_state = STATE_MOTOR_OFF
            # we do an action sequence to turn off the motors and stay in motor off  
            return "TURN_MOTORS_OFF"
        elif not self.blackboard.are_motors_on():
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
        if not self.blackboard.are_motors_available():
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
        if self.blackboard.is_robot_picked_up():
            self.blackboard.current_state = STATE_PICKED_UP
            # we do an action sequence to tgo to walkready and stay in picked up state            
            return "PICKED_UP"
        else:
            # robot is not picked up
            return "ON_GROUND"

    def get_reevaluate(self):
        return True


class Fallen(AbstractDecisionElement):
    """
    Decides if the robot is currently fallen and lying on the ground
    """

    def perform(self, reevaluate=False):
        # check if the robot is currently laying on the ground
        if self.blackboard.is_stand_up_active and self.blackboard.is_fallen():
            self.blackboard.current_state = STATE_FALLEN
            # we play a stand up animation            
            rospy.loginfo("FALLEN")
            return "FALLEN"
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
        if self.blackboard.falling_detection_active and self.blackboard.is_falling():
            self.blackboard.current_state = STATE_FALLING
            # we play a falling animation            
            return "FALLING"
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
        if self.blackboard.is_currently_walking():
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
        if not self.blackboard.is_walkready():
            self.blackboard.current_state = STATE_ANIMATION_RUNNING
            return "NOT_WALKREADY"
        else:
            self.blackboard.current_state = STATE_CONTROLABLE
            return "WALKREADY"

    def get_reevaluate(self):
        return True

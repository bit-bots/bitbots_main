import rospy 
from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
import humanoid_league_msgs.msg
from bitbots_hcm.hcm_stack_machine.hcm_connector import STATE_ANIMATION_RUNNING, STATE_CONTROLABLE, STATE_FALLEN, STATE_FALLING, STATE_HARDWARE_PROBLEM, STATE_MOTOR_OFF, STATE_PENALTY, STATE_PICKED_UP, STATE_RECORD, STATE_SHUT_DOWN, STATE_START_UP, STATE_WALKING
from bitbots_hcm.hcm_stack_machine.actions import WaitForIMU, WaitForMotors, StayShutDown, StayAnimationRunning, StayControlable, StayInPenalty, StayMotorsOff, StayPickedUp, StayRecord, StayWalking, PlayAnimationStandUp, PlayFallingAnimation, PlayPenaltyAnimation, PlayMotorOffAnimation
from bitbots_hcm.hcm_stack_machine.actions.change_motor_power import TurnMotorsOff, TurnMotorsOn
from bitbots_hcm.hcm_stack_machine.actions.turn_motors_soft import TurnMotorsSoft
from bitbots_hcm.hcm_stack_machine.actions.stop_walking import StopWalking

class StartHcm(AbstractDecisionElement):
    """
    Initializes HCM. 
    """

    def __init__(self, connector, _):
        super(StartHcm, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        if connector.shut_down_request:
            connector.current_state = STATE_SHUT_DOWN
            return self.push(StayShutDown)
        else:
            connector.current_state = STATE_START_UP
            return self.push(CheckIMU)

    def get_reevaluate(self):
        return True


class CheckIMU(AbstractDecisionElement):
    """
    Checks if we are getting information from the IMU.
    Since the HCM can not detect falls without it, we will shut everything down if we dont have an imu.
    """

    def __init__(self, connector, _):
        super(CheckIMU, self).__init__(connector)

    def perform(self, connector, reevaluate=False):      
        if connector.is_imu_timeout():
            # tell that we have a hardware problem
            connector.current_state = STATE_HARDWARE_PROBLEM
            # wait for IMU
            return self.push(WaitForIMU)
        return self.push(Penalty)

    def get_reevaluate(self):
        return True


class Penalty(AbstractDecisionElement):
    """
    Initializes HCM
    """

    def __init__(self, connector, _):
        super(Penalty, self).__init__(connector)

    def perform(self, connector, reevaluate=False):        
        if connector.penalized:
            connector.current_state = STATE_PENALTY
            # we do an action sequence to go into penalty and to stay there                        
            return self.push_action_sequence([PlayPenaltyAnimation, StayInPenalty])
        else:
            return self.push(MotorOffTimer)

    def get_reevaluate(self):
        return True


class MotorOffTimer(AbstractDecisionElement):
    """
    Decides on switching servo power
    """

    def __init__(self, connector, _):
        super(MotorOffTimer, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        # check if the time is reached
        if connector.is_motor_off_time():
            connector.current_state = STATE_MOTOR_OFF
            # we do an action sequence to turn off the motors and stay in motor off            
            return self.push_action_sequence([PlayMotorOffAnimation, TurnMotorsOff, StayMotorsOff])
        elif not connector.are_motors_on():
            # we have to turn the motors on
            return self.push(TurnMotorsOn)
        else:
            # motors are on and we can continue
            return self.push(CheckMotors)

    def get_reevaluate(self):
        return True


class CheckMotors(AbstractDecisionElement):
    """
    Checks if we are getting information from the motors.
    Since the HCM is not able to work without motor connection, we will stop if there are no values.
    """

    def __init__(self, connector, _):
        super(CheckMotors, self).__init__(connector)

    def perform(self, connector, reevaluate=False):      
        if not connector.are_motors_available():
            # tell that we have a hardware problem                            
            connector.current_state = STATE_HARDWARE_PROBLEM
            # wait for motors to connect
            return self.push(WaitForMotors)
        return self.push(Record)

    def get_reevaluate(self):
        return True


class Record(AbstractDecisionElement):
    """
    Decides if the robot is currently recording animations
    """

    def __init__(self, connector, _):
        super(Record, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        # check if the robot is currently recording animations
        if connector.record_active:                    
            connector.current_state = STATE_RECORD
            return self.push(StayRecord)
        else:
            # robot is not recording
            return self.push(PickedUp)

    def get_reevaluate(self):
        return True


class PickedUp(AbstractDecisionElement):
    """
    Decides if the robot is currently picked up
    """

    def __init__(self, connector, _):
        super(PickedUp, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        # check if the robot is currently beeing picked up
        if connector.robot_picked_up:
            connector.current_state = STATE_PICKED_UP
            # we do an action sequence to turn the motors to soft and stay in picked up state            
            return self.push_action_sequence([TurnMotorsSoft, StayPickedUp]) #TODO ist das sinnvoll auf soft zu gehen
        else:
            #TODO maybe we have to make the robot stiff again
            # robot is not picked up
            return self.push(Fallen)

    def get_reevaluate(self):
        return True


class Fallen(AbstractDecisionElement):
    """
    Decides if the robot is currently fallen and lying on the ground
    """

    def __init__(self, connector, _):
        super(Fallen, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        # check if the robot is currently laying on the ground
        if connector.is_stand_up_active and connector.is_fallen():
            connector.current_state = STATE_FALLEN
            # we play a stand up animation            
            return self.push(PlayAnimationStandUp)
        else:
            # robot is not fallen
            return self.push(Falling)

    def get_reevaluate(self):
        return True

class Falling(AbstractDecisionElement):
    """
    Decides if the robot is currently falling and has to act on this
    """

    def __init__(self, connector, _):
        super(Falling, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        # check if the robot is currently falling
        if connector.falling_detection_active and connector.is_falling():
            connector.current_state = STATE_FALLING
            # we play a stand up animation            
            return self.push(PlayFallingAnimation)
        else:
            # robot is not fallen
            return self.push(BehaviorAnimation)

    def get_reevaluate(self):
        return True


class BehaviorAnimation(AbstractDecisionElement):
    """
    Decides if the robot is currently wants to play an animation comming from the behavior
    """

    def __init__(self, connector, _):
        super(BehaviorAnimation, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        if connector.external_animation_running:
            connector.current_state = STATE_ANIMATION_RUNNING
            return self.push(StayAnimationRunning)
        else:
            return self.push(Walking)

    def get_reevaluate(self):
        return True


class Walking(AbstractDecisionElement):
    """
    Decides if the robot is currently walking
    """

    def __init__(self, connector, _):
        super(Walking, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        #TODO animation requested
        if connector.is_currently_walking():            
            connector.current_state = STATE_WALKING
            if connector.animation_requested:
                # we are walking but we have to stop to play an animation
                return self.push(StopWalking)
            else:                
                # we are walking and can stay like this
                return self.push(StayWalking)
        else:
            return self.push(Controlable)

    def get_reevaluate(self):
        return True


class Controlable(AbstractDecisionElement):
    """
    Decides if the robot is currently controlable
    """

    def __init__(self, connector, _):
        super(Controlable, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        # yes we are if we reached this decision        
        connector.current_state = STATE_CONTROLABLE
        return self.push(StayControlable)

    def get_reevaluate(self):
        return True

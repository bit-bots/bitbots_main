import rospy 
from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
import humanoid_league_msgs.msg
import bitbots_hcm.hcm_stack_machine.hcm_connector
from bitbots_hcm.hcm_stack_machine.actions import WaitForIMU, WaitForMotors, StayAnimationRunning, StayControlable, StayInPenalty, StayMotorsOff, StayPickedUp, StayRecord, StayWalking, PlayAnimationStandUp, PlayFallingAnimation, PlayPenaltyAnimation


class StartHcm(AbstractDecisionElement):
    """
    Initializes HCM. 
    """

    def __init__(self, connector, _):
        super(StartHcm, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        if connector.is_shutdown():
            connector.hcm.publish_state(STATE_SHUT_DOWN)          
            #TODO shutdown
        if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_START_UP)          
        return self.push(CheckIMU)

    def get_reevaluate(self):
        return False


class CheckIMU(AbstractDecisionElement):
    """
    Checks if we are getting information from the IMU.
    Since the HCM can not detect falls without it, we will shut everything down if we dont have an imu.
    """

    def __init__(self, connector, _):
        super(CheckIMU, self).__init__(connector)

    def perform(self, connector, reevaluate=False):      
        if not connector.hcm.is_imu_available():
            # tell that we have a hardware problem
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_HARDWARE_PROBLEM)
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
        if connector.hcm.is_penalty():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_PENALTY)
            # we do an action sequence to go into penalty and to stay there            
            self.push(StayInPenalty)
            return self.push(PlayPenaltyAnimation)
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
        if connector.hcm.is_motor_off_time():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_MOTOR_OFF)
            # we do an action sequence to turn off the motors and stay in motor off
            self.push(StayMotorsOff)
            return self.push(TurnMotorsOff)
        elif not connector.hcm.is_motor_off_time() and not connector.hcm.are_motors_on():
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
        if not connector.hcm.are_motors_available():
            # tell that we have a hardware problem
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_HARDWARE_PROBLEM)
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
        if connector.hcm.robot_picked_up():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_RECORD)            
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
        if connector.hcm.robot_picked_up():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_PICKED_UP)
            # we do an action sequence to turn the motors to soft and stay in picked up state
            self.push(StayPickedUp)
            return self.push(TurnMotorsSoft)
        else:
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
        if connector.hcm.is_stand_up_active() and connector.hcm.is_fallen():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_FALLEN)
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
        if connector.hcm.is_falling_detection_active() and connector.hcm.is_falling():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_FALLING)
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
        if connector.hcm.is_external_animation_running():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_ANIMATION_RUNNING)
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
        if connector.hcm.is_currently_walking():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_WALKING)  
            return self.push(StayWalking)
        else:
            return self.push(Controlable)

    def get_reevaluate(self):
        return True


class Controlable(AbstractDecisionElement):
    """
    Decides if the robot is currently walking
    """

    def __init__(self, connector, _):
        super(Controlable, self).__init__(connector)

    def perform(self, connector, reevaluate=False):        
        if not reevaluate:
            # only publish the new state if we are not reevaluating
            connector.hcm.publish_state(STATE_CONTROLABLE)  
        return self.push(StayControlable)

    def get_reevaluate(self):
        return True

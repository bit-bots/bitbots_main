import rospy
import math
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
import humanoid_league_msgs.msg
from bitbots_hcm.hcm_dsd.hcm_blackboard import STATE_ANIMATION_RUNNING, STATE_CONTROLLABLE, STATE_FALLEN, STATE_FALLING, \
    STATE_HARDWARE_PROBLEM, STATE_MOTOR_OFF, STATE_PENALTY, STATE_PICKED_UP, STATE_RECORD, STATE_SHUT_DOWN, \
    STATE_STARTUP, STATE_WALKING, STATE_HCM_OFF, STATE_KICKING
from humanoid_league_speaker.speaker import speak
from humanoid_league_msgs.msg import Audio


class StartHCM(AbstractDecisionElement):
    """
    Initializes HCM.
    """

    def perform(self, reevaluate=False):
        if self.blackboard.shut_down_request:
            if self.blackboard.current_state == STATE_HARDWARE_PROBLEM:
                self.blackboard.current_state = STATE_SHUT_DOWN
                return "SHUTDOWN_WHILE_HARDWARE_PROBLEM"
            else:
                self.blackboard.current_state = STATE_SHUT_DOWN
                return "SHUTDOWN_REQUESTED"
        else:
            if not reevaluate:
                self.blackboard.current_state = STATE_STARTUP
            return "RUNNING"

    def get_reevaluate(self):
        return True


class Stop(AbstractDecisionElement):
    """
    Handles manual stops
    """

    def perform(self, reevaluate=False):
        if self.blackboard.stopped:
            # we do an action sequence to go into stop and to stay there
            return "STOPPED"
        else:
            return "FREE"

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


class CheckMotors(AbstractDecisionElement):
    """
    Checks if we are getting information from the motors.
    Since the HCM is not able to work without motor connection, we will stop if there are no values.
    Needs to be checked before other sensors, since they also need the power to be able to response
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(CheckMotors, self).__init__(blackboard, dsd, parameters)
        self.last_msg = None
        self.last_different_msg_time = rospy.Time.from_sec(0)
        self.had_problem = False

    def perform(self, reevaluate=False):
        self.clear_debug_data()
        if self.blackboard.visualization_active:
            # we will have no problems with hardware in visualization
            return "OKAY"

        # we check if the values are actually changing, since the joint_state controller will publish the same message
        # even if there is no connection anymore. But we don't want to go directly to hardware error if we just
        # have a small break, since this can happen often due to loose cabling
        if self.last_msg is not None and self.blackboard.current_joint_state is not None \
                and not self.last_msg.position == self.blackboard.current_joint_state.position \
                and not self.blackboard.servo_diag_error:
            self.last_different_msg_time = self.blackboard.current_time
        self.last_msg = self.blackboard.current_joint_state

        if self.blackboard.simulation_active:
            # Some simulators will give exact same joint messages which look like errors, so ignore this case
            if self.last_msg:
                return "OKAY"
            else:
                return "MOTORS_NOT_STARTED"

        # check if we want to turn the motors off after not using them for a longer time
        if self.blackboard.last_motor_goal_time is not None \
                and self.blackboard.current_time.to_sec() - self.blackboard.last_motor_goal_time.to_sec() \
                > self.blackboard.motor_off_time:
            rospy.logwarn_throttle(5, "Didn't recieve goals for " + str(
                self.blackboard.motor_off_time) + " seconds. Will shut down the motors and wait for commands.")
            self.publish_debug_data("Time since last motor goals",
                                    self.blackboard.current_time.to_sec() - self.blackboard.last_motor_goal_time.to_sec())
            self.blackboard.current_state = STATE_MOTOR_OFF
            # we do an action sequence to turn off the motors and stay in motor off
            return "TURN_OFF"

        # see if we get no messages or always the exact same
        if self.blackboard.current_time.to_sec() - self.last_different_msg_time.to_sec() > 0.1:
            if self.blackboard.is_power_on:
                if self.blackboard.current_state == STATE_STARTUP and self.blackboard.current_time.to_sec() - \
                        self.blackboard.start_time.to_sec() < 10:
                    # we are still in startup phase, just wait and dont complain
                    return "MOTORS_NOT_STARTED"
                else:
                    # tell that we have a hardware problem
                    self.had_problem = True
                    # wait for motors to connect
                    self.blackboard.current_state = STATE_HARDWARE_PROBLEM
                    return "PROBLEM"
            else:
                # we have to turn the motors on
                return "TURN_ON"

        if self.had_problem:
            # had problem before, just tell that this is solved now
            rospy.loginfo("Motors are now connected. Will resume.")
            self.had_problem = False

        # motors are on and we can continue
        return "OKAY"

    def get_reevaluate(self):
        return True


class CheckIMU(AbstractDecisionElement):
    """
    Checks if we are getting information from the IMU.
    Since the HCM can not detect falls without it, we will shut everything down if we dont have an imu.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(CheckIMU, self).__init__(blackboard, dsd, parameters)
        self.last_msg = None
        self.last_different_msg_time = rospy.Time.from_sec(0)
        self.had_problem = False

    def perform(self, reevaluate=False):
        if self.blackboard.visualization_active:
            # In visualization, we do not have an IMU. Therefore, return OKAY to ignore that.
            return "OKAY"

        # we will get always the same message if there is no connection, so check if it differs
        if self.last_msg is not None and self.blackboard.imu_msg is not None \
                and not self.last_msg.orientation == self.blackboard.imu_msg.orientation \
                and not self.blackboard.imu_diag_error:
            self.last_different_msg_time = self.blackboard.current_time
        self.last_msg = self.blackboard.imu_msg

        if self.blackboard.simulation_active:
            # Some simulators will give exact same IMU messages which look like errors, so ignore this case
            if self.last_msg:
                return "OKAY"
            else:
                return "IMU_NOT_STARTED"

        if self.blackboard.current_time.to_sec() - self.last_different_msg_time.to_sec() > 0.1:
            if self.blackboard.current_state == STATE_STARTUP and self.blackboard.current_time.to_sec() - \
                    self.blackboard.start_time.to_sec() < 10:
                # wait for the IMU to start
                return "IMU_NOT_STARTED"
            else:
                self.blackboard.current_state = STATE_HARDWARE_PROBLEM
                self.had_problem = True
                return "PROBLEM"

        if self.had_problem:
            # had problem before, just tell that this is solved now
            rospy.loginfo("IMU is now connected. Will resume.")
            self.had_problem = False

        return "OKAY"

    def get_reevaluate(self):
        return True


class CheckPressureSensor(AbstractDecisionElement):
    """
    Checks connection to pressure sensors.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(CheckPressureSensor, self).__init__(blackboard, dsd, parameters)
        self.last_pressure_values = None
        self.last_different_msg_time = rospy.Time.from_sec(0)
        self.had_problem = False

    def perform(self, reevaluate=False):
        if self.blackboard.visualization_active:
            # no pressure sensors is visualization, but thats okay
            return "OKAY"

        if not self.blackboard.pressure_sensors_installed:
            # no pressure sensors installed, no check necessary
            return "OKAY"
        if not self.blackboard.pressure_diag_error:
            self.last_different_msg_time = self.blackboard.current_time

        if self.blackboard.current_time.to_sec() - self.last_different_msg_time.to_sec() > 0.1:
            if self.blackboard.current_state == STATE_STARTUP and self.blackboard.current_time.to_sec() - \
                    self.blackboard.start_time.to_sec() < 10:
                # wait for the pressure sensors to start
                self.blackboard.current_state = STATE_STARTUP
                return "PRESSURE_NOT_STARTED"
            else:
                self.blackboard.current_state = STATE_HARDWARE_PROBLEM
                return "PROBLEM"

        if self.had_problem:
            # had problem before, just tell that this is solved now
            rospy.loginfo("Pressure sensors are now connected. Will resume.")
            self.had_problem = False

        return "OKAY"

    def get_reevaluate(self):
        return True


class PickedUp(AbstractDecisionElement):
    """
    Decides if the robot is currently picked up
    """

    def perform(self, reevaluate=False):
        if self.blackboard.visualization_active:
            return "ON_GROUND"
        # check if the robot is currently being picked up. foot have no connection to the ground,
        # but robot is more or less upright (to differentiate from falling)
        if self.blackboard.pressure_sensors_installed and sum(self.blackboard.pressures) < 10 and \
                abs(self.blackboard.smooth_accel[0]) < self.blackboard.pickup_accel_threshold and \
                abs(self.blackboard.smooth_accel[1]) < self.blackboard.pickup_accel_threshold:
            self.blackboard.current_state = STATE_PICKED_UP
            if not reevaluate:
                speak("Picked up!", self.blackboard.speak_publisher, priority=50)
            # we do an action sequence to go to walkready and stay in picked up state
            return "PICKED_UP"

        # robot is not picked up
        return "ON_GROUND"

    def get_reevaluate(self):
        return True


class Falling(AbstractDecisionElement):
    """
    Decides if the robot is currently falling and has to act on this
    """

    def perform(self, reevaluate=False):
        # check if the robot is currently falling
        falling_direction = self.blackboard.fall_checker.check_falling(self.blackboard.gyro, self.blackboard.quaternion)
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
        # robot is not fallen
        return "NOT_FALLING"

    def get_reevaluate(self):
        return True


class FallingClassifier(AbstractDecisionElement):

    def perform(self, reevaluate=False):
        prediction = self.blackboard.classifier.smooth_classify(self.blackboard.imu_msg,
                                                                self.blackboard.current_joint_state,
                                                                self.blackboard.cop_l_msg, self.blackboard.cop_r_msg)
        if prediction == 0:
            return "NOT_FALLING"
        elif prediction == 1:
            return "FALLING_FRONT"
        elif prediction == 2:
            return "FALLING_BACK"
        elif prediction == 3:
            return "FALLING_LEFT"
        elif prediction == 4:
            return "FALLING_RIGHT"
        else:
            return "NOT_FALLING"

    def get_reevaluate(self):
        return True


class Sitting(AbstractDecisionElement):
    """
    Decides if the robot is sitting (due to sitting down earlier).
    """

    def perform(self, reevaluate=False):
        # simple check is looking at knee joint positions
        # todo can be done more sophisticated
        left_knee = 0
        right_knee = 0
        i = 0
        for joint_name in self.blackboard.current_joint_state.name:
            if joint_name == "LKnee":
                left_knee = self.blackboard.current_joint_state.position[i]
            elif joint_name == "RKnee":
                right_knee = self.blackboard.current_joint_state.position[i]
            i += 1

        if abs(left_knee) > 2.5 and abs(right_knee) > 2.5:
            return "YES"
        else:
            return "NO"

    def get_reevaluate(self):
        # we never have to reevaluate since this state of this can only be changed by decisions above it
        return False


class Fallen(AbstractDecisionElement):
    """
    Decides if the robot is fallen and lying on the ground
    """

    def perform(self, reevaluate=False):
        # check if the robot is currently laying on the ground
        fallen_side = self.blackboard.fall_checker.check_fallen(self.blackboard.smooth_accel, self.blackboard.gyro)
        if self.blackboard.is_stand_up_active and fallen_side is not None:
            self.blackboard.current_state = STATE_FALLEN
            # TODO
            self.blackboard.hacky_sequence_dynup_running = True
            # we play a stand up animation
            if fallen_side == self.blackboard.fall_checker.FRONT:
                return "FALLEN_FRONT"
            if fallen_side == self.blackboard.fall_checker.BACK:
                return "FALLEN_BACK"
            if fallen_side == self.blackboard.fall_checker.RIGHT:
                return "FALLEN_RIGHT"
            if fallen_side == self.blackboard.fall_checker.LEFT:
                return "FALLEN_LEFT"
        else:
            # robot is not fallen
            return "NOT_FALLEN"

    def get_reevaluate(self):
        return not self.blackboard.hacky_sequence_dynup_running


class ExternalAnimation(AbstractDecisionElement):
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
            self.blackboard.current_state = STATE_CONTROLLABLE
            return "WALKREADY"

    def is_walkready(self):
        """
        We check if any joint is has an offset from the walkready pose which is higher than a threshold
        """
        if self.blackboard.current_joint_state is None:
            return False
        i = 0
        for joint_name in self.blackboard.current_joint_state.name:
            if joint_name == "HeadPan" or joint_name == "HeadTilt":
                # we dont care about the head position
                i += 1
                continue
            if abs(math.degrees(self.blackboard.current_joint_state.position[i]) -
                   self.blackboard.walkready_pose_dict[joint_name]) > self.blackboard.walkready_pose_threshold:
                return False
            i += 1
        return True

    def get_reevaluate(self):
        return True


class Kicking(AbstractDecisionElement):
    """
    Decides if the robot is currently kicking
    """

    def perform(self, reevaluate=False):
        if self.blackboard.last_kick_feedback is not None and \
                (rospy.Time.now() - self.blackboard.last_kick_feedback) < rospy.Duration.from_sec(1):
            self.blackboard.current_state = STATE_KICKING
            return 'KICKING'
        else:
            return 'NOT_KICKING'

    def get_reevaluate(self):
        return True

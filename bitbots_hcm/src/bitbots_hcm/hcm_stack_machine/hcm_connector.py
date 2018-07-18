from bitbots_connector.connector import AbstractConnector
import numpy
import rospy
from bitbots_hcm import FallChecker


# robot states that are published to the rest of the software
# definition from humanoid_league_msgs/RobotControlState.msg
STATE_CONTROLABLE = 0
STATE_FALLING = 1
STATE_FALLEN = 2
STATE_GETTING_UP = 3
STATE_ANIMATION_RUNNING = 4
STATE_STARTUP = 5
STATE_SHUT_DOWN = 6
STATE_PENALTY = 7
STATE_PENALTY_ANIMATION = 8
STATE_RECORD = 9
STATE_WALKING = 10
STATE_MOTOR_OFF=11
STATE_HCM_OFF=12
STATE_HARDWARE_PROBLEM=13
STATE_PICKED_UP = 14


class HcmConnector(AbstractConnector):
    def __init__(self):
        super(HcmConnector, self).__init__()

        self.current_state = STATE_STARTUP        
        self.penalized = False
        self.shut_down_request = False

        # this is used to prevent calling rospy.Time a lot, which takes some time
        # we assume that the time does not change during one update cycle
        self.current_time = 0

        # Imu
        self.last_imu_update_time = 0
        self.imu_timeout_duration = rospy.get_param("imu_timeout_duration")
        self.accel = numpy.array([0, 0, 0])
        self.gyro = numpy.array([0, 0, 0])
        self.smooth_accel = numpy.array([0, 0, 0])
        self.smooth_gyro = numpy.array([0, 0, 0])
        self.not_much_smoothed_gyro = numpy.array([0, 0, 0])
        self.quaternion  = numpy.array([0,0,0,0.21])

        # Animation
        self.animation_action_client = None
        self.last_animation_goal_time = 0
        self.external_animation_running = True

        self.last_walking_goal_time = 0

        self.record_active = False

        self.fall_checker = FallChecker()
        
        self.simulation_active = rospy.get_param("simulation_active")
        self.motor_off_time = rospy.get_param("hcm/motor_off_time")        
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def is_imu_timeout(self):
        """
        Havent we recieved updates from the imu in the last time
        """
        return self.current_time - self.last_imu_update_time > self.imu_timeout_duration
    
    def is_motor_timeout(self):
        """
        Havent we recieved updates from the motors in the last time
        """
        return self.current_time - self.last_imu_update_time > self.imu_timeout_duration
    
    def is_motor_off_time(self):
        """
        After a duration without any commands, the motors should go off for safty reasons, e.g. user forgot to turn off robot
        """
        return self.current_time - self.last_motor_goal_time > self.motor_off_time
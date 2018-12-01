from bitbots_connector.abstract_connector import AbstractConnector
import numpy
import rospy
import math
import json
import rospkg
import os
from bitbots_hcm.fall_checker import FallChecker
from geometry_msgs.msg import Twist
from bitbots_stackmachine.stack_machine import StackMachine
from humanoid_league_msgs.msg import RobotControlState


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
        self.simulation_active = rospy.get_param("/simulation_active")

        # this is used to prevent calling rospy.Time a lot, which takes some time
        # we assume that the time does not change during one update cycle
        self.current_time = rospy.Time()

        # Imu
        self.last_imu_update_time = rospy.Time()
        self.imu_timeout_duration = rospy.get_param("hcm/imu_timeout_duration")
        self.accel = numpy.array([0, 0, 0])
        self.gyro = numpy.array([0, 0, 0])
        self.smooth_accel = numpy.array([0, 0, 0])
        self.smooth_gyro = numpy.array([0, 0, 0])
        self.not_much_smoothed_gyro = numpy.array([0, 0, 0])
        self.quaternion  = numpy.array([0,0,0,0.21])

        # Animation
        self.animation_action_client = None
        self.last_animation_goal_time = rospy.Time()
        self.external_animation_running = False
        self.animation_requested = False
        self.hcm_animation_finished = False
        self.walkready_animation = rospy.get_param("hcm/animations/walkready")
        self.falling_animation_front = rospy.get_param("hcm/animations/falling_front")
        self.falling_animation_back = rospy.get_param("hcm/animations/falling_back")
        self.falling_animation_left = rospy.get_param("hcm/animations/falling_left")
        self.falling_animation_right = rospy.get_param("hcm/animations/falling_right")
        self.penalty_animation = rospy.get_param("hcm/animations/penalty")
        self.sit_down_animation = rospy.get_param("hcm/animations/sit_down")
        self.motor_off_animation = rospy.get_param("hcm/animations/motor_off")
        self.stand_up_front_animation = rospy.get_param("hcm/animations/stand_up_front")
        self.stand_up_back_animation = rospy.get_param("hcm/animations/stand_up_back")
        self.stand_up_side_animation = rospy.get_param("hcm/animations/stand_up_side")

        # motors
        self.last_motor_goal_time = rospy.Time.now() # initilize with current time, or motors will be turned off on start
        self.last_motor_update_time = rospy.Time()
        self.motor_timeout_duration = rospy.get_param("hcm/motor_timeout_duration")
        self.motor_off_time = rospy.get_param("hcm/motor_off_time")
        self.current_joint_positions = None
        anim_package = rospy.get_param("hcm/animations/anim_package")
        rospack = rospkg.RosPack()
        path = rospack.get_path(anim_package)
        path = os.path.join(path, 'animations/motion/', self.walkready_animation +'.json')
        with open(path, 'r') as f:
            json_data = json.load(f)
        keyframes = json_data["keyframes"]
        self.walkready_pose_dict = keyframes[-1]["goals"]
        self.walkready_pose_threshold = rospy.get_param("hcm/animations/walkready_pose_threshold")

        # walking
        self.last_walking_goal_time = rospy.Time()
        self.walk_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.record_active = False

        # falling
        self.fall_checker = FallChecker()
        self.is_stand_up_active = not self.simulation_active and rospy.get_param("hcm/stand_up_active", False) 
        self.falling_detection_active = not self.simulation_active and rospy.get_param("hcm/falling_active", False)             

    def is_imu_timeout(self):
        """
        Havent we recieved updates from the imu in the last time
        """
        return self.current_time.to_sec() - self.last_imu_update_time.to_sec() > self.imu_timeout_duration
    
    def is_motor_timeout(self):
        """
        Havent we recieved updates from the motors in the last time
        """
        return self.current_time.to_sec() - self.last_motor_update_time.to_sec() > self.motor_timeout_duration
    
    def is_motor_off_time(self):
        """
        After a duration without any commands, the motors should go off for safty reasons, e.g. user forgot to turn off robot
        """
        return self.current_time.to_sec() - self.last_motor_goal_time.to_sec() > self.motor_off_time

    def are_motors_on(self):
        """
        """
        return self.current_time.to_sec() - self.last_motor_update_time.to_sec() < 0.1

    def are_motors_available(self):
        """
        """
        return self.current_time.to_sec() - self.last_motor_update_time.to_sec() < 0.1

    def is_robot_picked_up(self):
        return False #TODO needs feet sensors
    
    def is_currently_walking(self):
        return self.current_time.to_sec() - self.last_walking_goal_time.to_sec() < 0.1

    def is_walkready(self):
        """
        We check if any joint is has an offset from the walkready pose which is higher than a threshold
        """
        if self.current_joint_positions is None:
            return False
        i = 0
        for joint_name in self.current_joint_positions.name:
            if joint_name == "HeadPan" or joint_name == "HeadTilt":
                #we dont care about the head position
                continue
            if abs(math.degrees(self.current_joint_positions.position[i]) - self.walkready_pose_dict[joint_name]) > self.walkready_pose_threshold:
                return False
            i +=1 
        return True

    def robot_falling_direction(self):
        return self.fall_checker.check_falling(self.gyro, self.quaternion)

    def is_falling(self):
        if self.robot_falling_direction() is None:
            return False
        return True

    def get_fallen_side(self):
        return self.fall_checker.check_fallen(self.smooth_accel, self.gyro)

    def is_fallen(self):
        if self.get_fallen_side() is None:
            return False
        return True

    def stop_walking(self):
        msg= Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.angular.z = 0
        self.walk_pub.publish(msg)
import time
import numpy
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import math
import json
import rospkg
import os

from actionlib_msgs.msg import GoalID
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_srvs.srv import Empty
from bitbots_hcm.fall_checker import FallChecker
from geometry_msgs.msg import Twist
from humanoid_league_msgs.msg import RobotControlState
from bitbots_hcm.fall_classifier import FallClassifier
from ament_index_python import get_package_share_directory
from std_srvs.srv import SetBool


class HcmBlackboard():
    def __init__(self, node:Node):
        self.node = node
        self.current_state = RobotControlState.STARTUP
        self.stopped = False
        self.shut_down_request = False
        self.simulation_active = self.node.get_parameter('simulation_active').get_parameter_value().bool_value
        self.visualization_active = self.node.get_parameter('visualization_active').get_parameter_value().bool_value

        # this is used to prevent calling rospy.Time a lot, which takes some time
        # we assume that the time does not change during one update cycle
        self.current_time = self.node.get_clock().now()
        self.start_time = self.current_time
        # Imu
        self.last_imu_update_time = None
        self.imu_timeout_duration = self.node.get_parameter("imu_timeout_duration")
        self.accel = numpy.array([0, 0, 0])
        self.gyro = numpy.array([0, 0, 0])
        self.smooth_accel = numpy.array([0, 0, 0])
        self.smooth_gyro = numpy.array([0, 0, 0])
        self.not_much_smoothed_gyro = numpy.array([0, 0, 0])
        self.quaternion = numpy.array([0, 0, 0, 0.21])
        self.pickup_accel_threshold = 1000

        # Pressure sensors
        self.pressure_sensors_installed = self.node.get_parameter('pressure_sensors_installed').get_parameter_value().bool_value
        self.pressure_timeout_duration = self.node.get_parameter("pressure_timeout_duration").get_parameter_value().double_value
        self.last_pressure_update_time = None
        # initialize values high to prevent wrongly thinking the robot is picked up during start or in simulation
        self.pressures = [100] * 8
        foot_zero_service_name = self.node.get_parameter("foot_zero_service").get_parameter_value().string_value
        self.foot_zero_service = self.node.create_client(Empty, foot_zero_service_name)

        self.motor_switch_service = self.node.create_client(SetBool, 'core/switch_power')


        # Animation
        self.animation_action_client = None
        self.animation_action_current_goal = None
        self.dynup_action_client = None
        self.dynup_action_current_goal = None
        self.last_animation_goal_time = self.node.get_clock().now()
        self.external_animation_running = False
        self.animation_requested = False
        self.hcm_animation_finished = False
        self.walkready_animation = self.node.get_parameter("animations.walkready").get_parameter_value().string_value
        if self.simulation_active:
            self.walkready_animation = self.node.get_parameter("animations.walkready_sim").get_parameter_value().string_value
        self.falling_animation_front = self.node.get_parameter("animations.falling_front").get_parameter_value().string_value
        self.falling_animation_back = self.node.get_parameter("animations.falling_back").get_parameter_value().string_value
        self.falling_animation_left = self.node.get_parameter("animations.falling_left").get_parameter_value().string_value
        self.falling_animation_right = self.node.get_parameter("animations.falling_right").get_parameter_value().string_value
        self.stop_animation = self.node.get_parameter("animations.penalty").get_parameter_value().string_value
        self.sit_down_animation = self.node.get_parameter("animations.sit_down").get_parameter_value().string_value
        self.motor_off_animation = self.node.get_parameter("animations.motor_off").get_parameter_value().string_value
        self.stand_up_front_animation = self.node.get_parameter("animations.stand_up_front").get_parameter_value().string_value
        self.stand_up_back_animation = self.node.get_parameter("animations.stand_up_back").get_parameter_value().string_value
        self.stand_up_left_animation = self.node.get_parameter("animations.stand_up_left").get_parameter_value().string_value
        self.stand_up_right_animation = self.node.get_parameter("animations.stand_up_right").get_parameter_value().string_value
        # motors
        # initialize with current time, or motors will be turned off on start
        self.last_motor_goal_time = self.node.get_clock().now()
        self.last_motor_update_time =  Time(seconds=int(0), nanoseconds=0)
        self.motor_timeout_duration = self.node.get_parameter("motor_timeout_duration").get_parameter_value().double_value
        self.motor_off_time = self.node.get_parameter("motor_off_time").get_parameter_value().double_value
        self.current_joint_state = None
        self.previous_joint_state = None
        anim_package = self.node.get_parameter("animations.anim_package").get_parameter_value().string_value
        path = get_package_share_directory(anim_package)
        path = os.path.join(path, 'animations/motion/', self.walkready_animation + '.json')
        with open(path, 'r') as f:
            json_data = json.load(f)
        keyframes = json_data["keyframes"]
        self.walkready_pose_dict = keyframes[-1]["goals"]
        self.walkready_pose_threshold = self.node.get_parameter("animations.walkready_pose_threshold").get_parameter_value().double_value
        self.is_power_on = False
        # walking
        self.last_walking_goal_time = self.node.get_clock().now()
        self.walk_pub = self.node.create_publisher(Twist, "cmd_vel", 1)

        self.record_active = False

        # falling
        self.fall_checker = FallChecker(self.node)
        self.is_stand_up_active = self.node.get_parameter('stand_up_active').get_parameter_value().bool_value
        self.falling_detection_active = self.node.get_parameter('falling_active').get_parameter_value().bool_value

        # kicking
        self.last_kick_feedback = None  # type: rospy.Time

        # direct messages for falling classier
        # todo needs refactoring
        path = get_package_share_directory('bitbots_hcm')
        smooth_threshold = self.node.get_parameter('smooth_threshold').get_parameter_value().double_value
        self.classifier = FallClassifier(path + "/classifier/", smooth_threshold=smooth_threshold)
        self.imu_msg = None
        self.cop_l_msg = None
        self.cop_r_msg = None

        self.servo_diag_error = False
        self.imu_diag_error = False
        self.pressure_diag_error = False

        self.move_base_cancel_pub = self.node.create_publisher(GoalID, "move_base/cancel", 1)

    def diag_cb(self, msg: DiagnosticArray):
        for status in msg.status:
            if status.name == "/Servos":
                self.servo_diag_error = status.level == DiagnosticStatus.ERROR or status.level == DiagnosticStatus.STALE
            elif status.name == "/IMU":
                self.imu_diag_error = status.level == DiagnosticStatus.ERROR or status.level == DiagnosticStatus.STALE
            elif status.name == "/Pressure":
                self.pressure_diag_error = status.level == DiagnosticStatus.ERROR or status.level == DiagnosticStatus.STALE

    def last_kick_feedback_callback(self, msg):
        self.last_kick_feedback = self.node.get_clock().now()

    def cancel_move_base_goal(self):
        self.move_base_cancel_pub.publish(GoalID())

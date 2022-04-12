import numpy
import rclpy
from rclpy.node import Node
import math
import json
import rospkg
import os

from actionlib_msgs.msg import GoalID
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_srvs.srv import Empty
from bitbots_hcm.fall_checker import FallChecker
from geometry_msgs.msg import Twist
from bitbots_msgs.msg import KickActionFeedback

from humanoid_league_msgs.msg import RobotControlState
from bitbots_hcm.fall_classifier import FallClassifier
import rospkg


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
        self.imu_timeout_duration = self.node.get_parameter("hcm/imu_timeout_duration")
        self.accel = numpy.array([0, 0, 0])
        self.gyro = numpy.array([0, 0, 0])
        self.smooth_accel = numpy.array([0, 0, 0])
        self.smooth_gyro = numpy.array([0, 0, 0])
        self.not_much_smoothed_gyro = numpy.array([0, 0, 0])
        self.quaternion = numpy.array([0, 0, 0, 0.21])
        self.pickup_accel_threshold = 1000

        # Pressure sensors
        self.pressure_sensors_installed = self.node.get_parameter('hcm/pressure_sensors_installed').get_parameter_value().double_value
        self.pressure_timeout_duration = self.node.get_parameter("hcm/pressure_timeout_duration")
        self.last_pressure_update_time = None
        # initialize values high to prevent wrongly thinking the robot is picked up during start or in simulation
        self.pressures = [100] * 8
        foot_zero_service_name = self.node.get_parameter("hcm/foot_zero_service")
        self.foot_zero_service = self.node.create_client(Empty, foot_zero_service_name)

        # Animation
        self.animation_action_client = None
        self.dynup_action_client = None
        self.dynamic_kick_client = None
        self.last_animation_goal_time = rospy.Time()
        self.external_animation_running = False
        self.animation_requested = False
        self.hcm_animation_finished = False
        self.walkready_animation = rospy.get_param("hcm/animations/walkready")
        if rospy.get_param("/simulation_active"):
            self.walkready_animation = rospy.get_param("hcm/animations/walkready_sim")
        self.falling_animation_front = rospy.get_param("hcm/animations/falling_front")
        self.falling_animation_back = rospy.get_param("hcm/animations/falling_back")
        self.falling_animation_left = rospy.get_param("hcm/animations/falling_left")
        self.falling_animation_right = rospy.get_param("hcm/animations/falling_right")
        self.stop_animation = rospy.get_param("hcm/animations/penalty")
        self.sit_down_animation = rospy.get_param("hcm/animations/sit_down")
        self.motor_off_animation = rospy.get_param("hcm/animations/motor_off")
        self.stand_up_front_animation = rospy.get_param("hcm/animations/stand_up_front")
        self.stand_up_back_animation = rospy.get_param("hcm/animations/stand_up_back")
        self.stand_up_left_animation = rospy.get_param("hcm/animations/stand_up_left")
        self.stand_up_right_animation = rospy.get_param("hcm/animations/stand_up_right")

        # motors
        # initialize with current time, or motors will be turned off on start
        self.last_motor_goal_time = self.get_clock().now()
        self.last_motor_update_time = Time(seconds=int(0), nanoseconds=0 % 1 * 1e9)
        self.motor_timeout_duration = rospy.get_param("hcm/motor_timeout_duration")
        self.motor_off_time = rospy.get_param("hcm/motor_off_time")
        self.current_joint_state = None
        self.previous_joint_state = None
        anim_package = rospy.get_param("hcm/animations/anim_package")
        rospack = rospkg.RosPack()
        path = rospack.get_path(anim_package)
        path = os.path.join(path, 'animations/motion/', self.walkready_animation + '.json')
        with open(path, 'r') as f:
            json_data = json.load(f)
        keyframes = json_data["keyframes"]
        self.walkready_pose_dict = keyframes[-1]["goals"]
        self.walkready_pose_threshold = rospy.get_param("hcm/animations/walkready_pose_threshold")
        self.is_power_on = False

        # walking
        self.last_walking_goal_time = rospy.Time()
        self.walk_pub = self.create_publisher(Twist, "cmd_vel", 1)

        self.record_active = False

        # falling
        self.fall_checker = FallChecker()
        self.is_stand_up_active = self.get_parameter('"hcm/stand_up_active"').get_parameter_value().double_value
        self.falling_detection_active = self.get_parameter('"hcm/falling_active"').get_parameter_value().double_value

        # kicking
        self.last_kick_feedback = None  # type: rospy.Time

        # direct messages for falling classier
        # todo needs refactoring

        rospack = rospkg.RosPack()
        rospack.list()
        path = rospack.get_path('bitbots_hcm')
        smooth_threshold = self.get_parameter('"hcm/smooth_threshold"').get_parameter_value().double_value
        self.classifier = FallClassifier(path + "/src/bitbots_hcm/classifier/", smooth_threshold=smooth_threshold)
        self.imu_msg = None
        self.cop_l_msg = None
        self.cop_r_msg = None

        def last_kick_feedback_callback(msg):
            self.last_kick_feedback = self.get_clock().now()

        rospy.Subscriber('dynamic_kick/feedback', KickActionFeedback, last_kick_feedback_callback, tcp_nodelay=True,
                         queue_size=1)

        self.servo_diag_error = False
        self.imu_diag_error = False
        self.pressure_diag_error = False

        def diag_cb(msg: DiagnosticArray):
            for status in msg.status:
                if status.name == "/Servos":
                    self.servo_diag_error = status.level == DiagnosticStatus.ERROR or status.level == DiagnosticStatus.STALE
                elif status.name == "/IMU":
                    self.imu_diag_error = status.level == DiagnosticStatus.ERROR or status.level == DiagnosticStatus.STALE
                elif status.name == "/Pressure":
                    self.pressure_diag_error = status.level == DiagnosticStatus.ERROR or status.level == DiagnosticStatus.STALE

        rospy.Subscriber("/diagnostics_agg", DiagnosticArray, diag_cb, tcp_nodelay=True, queue_size=1)

        self.move_base_cancel_pub = self.create_publisher(GoalID, "move_base/cancel", 1)

    def cancel_move_base_goal(self):
        self.move_base_cancel_pub.publish(GoalID())


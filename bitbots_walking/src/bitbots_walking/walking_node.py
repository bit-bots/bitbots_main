#!/usr/bin/env python3
import threading

import rospy
import time
from bitbots_common.pose.pypose import PyPose as Pose
from bitbots_common.util.pose_util import set_joint_state_on_pose
from geometry_msgs.msg import Twist
from humanoid_league_msgs.msg import MotionState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from bitbots_common.util.pose_to_message import pose_goal_to_traj_msg

from bitbots_walking.zmpwalking import ZMPWalkingEngine


class WalkingNode(object):
    """ This node computes walking positions based on the current position, if the walking is active."""

    def __init__(self):
        rospy.init_node("bitbots_walking", anonymous=False)

        # --- Params ---
        zmp_config = rospy.get_param("/ZMPConfig/" + rospy.get_param("/robot_type_name"))
        self.with_gyro = zmp_config["use_gyro_for_walking"]
        robot_type_name = rospy.get_param("/robot_type_name")
        self.used_motor_cids = rospy.get_param("/cm730/" + robot_type_name + "/motors")
        self.used_motor_names = Pose().get_joint_names_cids(self.used_motor_cids[0:-2])  # without head

        # --- Class Variables ---
        self.walking = ZMPWalkingEngine()
        self.walkready_animation = self.walking.create_walkready_pose(duration=1.5)
        self.current_pose = Pose()
        self.goal_pose = Pose()
        self.pose_lock = threading.Lock()

        self.accel = (0, 0, 0)
        self.gyro = (0, 0, 0)

        self.walk_active = False
        self.walk_forward = 0
        self.walk_sideward = 0
        self.walk_angular = 0

        self.motion_state = None
        self.walking_started = 0

        # --- pre defiened messages for performance ---
        self.traj_msg = JointTrajectory()
        self.traj_msg.joint_names = [x.decode() for x in self.used_motor_names]
        self.traj_point = JointTrajectoryPoint()

        # --- Initialize Topics ---
        self.odometry_publisher = rospy.Publisher("/odometry", Odometry, queue_size=10)
        self.motor_goal_publisher = rospy.Publisher("/walking_motor_goals", JointTrajectory,
                                                    queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        rospy.Subscriber("/motion_state", MotionState, self.motion_state_cb)
        rospy.Subscriber("/joint_states", JointState, self.current_position_cb)
        rospy.Subscriber("/imu", Imu, self.imu_cb)

        # --- Start loop ---
        self.run()

    def cmd_vel_cb(self, msg):
        # we use only 3 values from the twist messages, as the robot is not capable of jumping or spinning around its
        # axis
        self.walk_forward = msg.linear.x
        self.walk_sideward = msg.linear.y
        self.walk_angular = msg.angular.z
        # deactivate walking if goal is 0 movement, else activate it
        self.walk_active = not (self.walk_forward == 0 and self.walk_sideward == 0 and self.walk_angular == 0)

    def motion_state_cb(self, msg):
        self.motion_state = msg.state

    def current_position_cb(self, msg):
        # we lock the pose to make shure nobody is reading it at the same time
        self.pose_lock.acquire()
        names = [x.encode("utf-8") for x in msg.name]
        self.current_pose.set_positions_rad(names, list(msg.position))
        # self.current_pose.set_speeds(names, list(msg.velocity))
        self.pose_lock.release()

    def imu_cb(self, msg):
        self.accel = msg.linear_acceleration
        self.gyro = msg.angular_velocity

    def calculate_walking(self):
        """
        Set parameters for walking and calculate pose.
        """

        # Aktuelle geschwindigkeitswerte Setzen
        self.walking.set_velocity(
            self.walk_forward / 150.0,
            self.walk_sideward / 50.0,
            self.walk_angular / 50.0)  # werte aus config erstmal hard TODO dyn conf
        # Gyro auslesen und an das Walking weitergeben
        if self.with_gyro is True:
            rospy.logwarn("Your trying to use the gyro with walking. The values are now in rad/sec (ROS standard) and "
                          "not the cm730 specific units. Please convert theme or adapt the walking algorithm "
                          "acordingly. It's propably not going to work like this.")
            gyro_x, gyro_y, gyro_z = self.gyro
            self.walking.set_gyro(gyro_x, gyro_y, gyro_z)  ###gyro
        # Pose berechnen
        self.zmp_foot_phase = self.walking.process()
        # todo check if this is still in use
        # self.ipc.set_walking_foot_phase(self.zmp_foot_phase)

        return self.walking.pose

    def walking_start(self):
        """
        This is starting the walking.
        """
        self.walking.start()
        self.walking.stance_reset()

    def walking_reset(self):
        """
        Resets the walking and stops it *imediatly*. This means that it can also stop during a step, thus in an
        unstable position. Should be normally used when the robot is already falling.
        """
        self.walking.stop()
        self.walking.stance_reset()

    def walking_stop(self):
        """
        Tells the walking that we want to stop. The walking will run a bit more until it reaches a stable stopping
        position. Therefore :func:`calculate_walking` has to be called further on.
        """
        self.walking.stop()

    def publish_motor_goals(self):
        msg = pose_goal_to_traj_msg(self.goal_pose, self.used_motor_names, self.traj_msg, self.traj_point)
        self.motor_goal_publisher.publish(msg)
        #rospy.logerr("pub")

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.walking.running:
                # The walking is walking
                if self.motion_state == MotionState.WALKING or (
                        self.motion_state == MotionState.CONTROLABLE and time.time() - self.walking_started < 1):
                    # The robot is in the right state, let's compute next pose
                    # if we just started walking, the motion does maybe not know it yet
                    if not self.walk_active:
                        # the client told us to stop, so let's stop
                        self.walking_stop()
                        # After this we still have to compute the next pose, because we have to do a fex steps to stop
                    self.pose_lock.acquire()
                    self.goal_pose.update(self.calculate_walking())
                    self.pose_lock.release()
                    self.publish_motor_goals()
                else:
                    # The motion changed from state walking to something else.
                    # das bedeutet das wir von der Motion gezwungen wurden
                    # etwas anderes zu tun (z.B. Aufstehn/Hinfallen)
                    # Dann reseten Wir das Walking und stoppen dabei.
                    self.walking_reset()
                    # reset pid, if the walking did something with this
                    p = rospy.get_param("/mx28config/RAM/p")
                    i = rospy.get_param("/mx28config/RAM/i")
                    d = rospy.get_param("/mx28config/RAM/d")
                    for name, joint in self.goal_pose.joints:
                        joint.p = p
                        joint.i = i
                        joint.d = d
            else:
                # We're not currently running, test if we want to start
                if self.walk_active and self.motion_state in (
                        MotionState.CONTROLABLE, MotionState.WALKING, MotionState.MOTOR_OFF):
                    rospy.logwarn("started walking")
                    self.walking_started = time.time()
                    self.walking_start()
            rate.sleep()


if __name__ == "__main__":
    walking = WalkingNode()
    rospy.spin()

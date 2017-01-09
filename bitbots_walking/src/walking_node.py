#!/usr/bin/env python
#-*- coding:utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from humanoid_league_msgs.msg import MotionState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from trajectory_msgs.msg import JointTrajectory
from bitbots_common.pose.pypose import PyPose as Pose
from bitbots_common.util.pose_util import set_joint_state_on_pose

from bitbots_walking.src.zmpwalking import ZMPWalkingEngine


class WalkingNode(object):
    def __init__(self):
        self.current_pose = Pose()
        self.goal_pose = Pose()
        self.accel = (0, 0, 0)
        self.gyro = (0, 0, 0)
        self.motion_state = None
        self.walking_started = 0

        zmp_config = rospy.get_param("/ZMPConfig" + rospy.get_param("/RobotTypeName"))
        self.walking = ZMPWalkingEngine()
        self.walkready_animation = self.walking.create_walkready_pose(duration=1.5)

        self.with_gyro = zmp_config["use_gyro_for_walking"]

        self.walk_active = False
        self.walk_forward = 0
        self.walk_sideward = 0
        self.walk_angular = 0

        rospy.init_node("bitbots_walking", anonymous=False)
        self.motor_goal_publisher = rospy.Publisher("/walking_motor_goals", JointTrajectory)
        self.odometry_publisher = rospy.Publisher("/odometry", Odometry)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        rospy.Subscriber("/motion_state", MotionState, self.motion_state_cb)
        rospy.Subscriber("/motor_current_position", JointState, self.current_position_cb)
        rospy.Subscriber("/imu", Imu, self.imu_cb)

        self.run()

    def cmd_vel_cb(self, msg):
        # we use only 3 values from the twist messages, as the robot is not capable of jumping or spinning around its axis
        self.walk_forward = msg.linear.x
        self.walk_sideward = msg.linear.y
        self.walk_angular = msg.angular.z
        # deactivate walking if goal is 0 movement, else activate it
        self.walk_active = self.walk_forward == 0 and self.walk_sideward == 0 and self.walk_angular == 0

    def motion_state_cb(self, msg):
        self.motion_state = msg.state

    def current_position_cb(self, msg):
        set_joint_state_on_pose(msg, self.current_pose)

    def imu_cb(self, msg):
        self.gyro = msg.linear_velocity
        self.accel = msg.angular_velocity

    def calculate_walking(self):
        """
        Setzt die Parameter fürs walking und berechnet die Pose
        """

        # Aktuelle geschwindigkeitswerte Setzen
        self.walking.set_velocity(
            self.walk_forward / 150.0,
            self.walk_sideward / 50.0,
            self.walk_angular / 50.0)  # werte aus config erstmal hard TODO
        # Gyro auslesen und an das Walking weitergeben
        if self.with_gyro is True:
            gyro_x, gyro_y, gyro_z = self.gyro
            self.walking.set_gyro(gyro_x, gyro_y, gyro_z)  ###gyro
        # Pose berechnen
        self.zmp_foot_phase = self.walking.process()
        # todo check if this is still in use
        # self.ipc.set_walking_foot_phase(self.zmp_foot_phase)

        # Pose zurückgeben
        return self.walking.pose

    def walking_start(self):
        """
        Startet das Walking. Ab sofort wir der Roboter laufen.
        """
        self.walking.start()
        self.walking.stance_reset()

    def walking_reset(self):
        """
        Resettet das Walking und stoppt es dabei *sofort*. Das bedeutet
        das es auch mitten im Schritt in einer instabilen Position anhalten
        kann. Überwiegend zum abbrechen wenn wir gerade Fallen oder
        ähnliches
        """
        self.walking.stop()
        self.walking.stance_reset()

    def walking_stop(self):
        """
        Sagt dem Walking das wir stoppen wollen. Das Walking wird noch
        etwas Weiterlaufen bis es einen Stabilen zustand zum anhalten
        erreicht hat, daher muss weiter :func:`calculate_walking`
        benutzt werden
        """
        self.walking.stop()

    def run(self):
        if self.walking.running:
            # The walking is walking
            if self.motion_state == MotionState.WALKING:
                # The robot is in the right state, let's compute next pose
                if not self.walk_active:
                    # the client told us to stop, so let's stop
                    self.walking_stop()
                    # After this we still have to compute the next pose, because we have to do a fex steps to stop
                self.goal_pose.update(self.calculate_walking())
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
                self.walking_started = rospy.Time.now()
                self.walking_start()

if __name__ == "__main__":
    walking = WalkingNode()
    rospy.spin()
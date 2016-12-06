#!/usr/bin/env python
#-*- coding:utf-8 -*-
import time

import math
from Cython.Includes.cpython.exc import PyErr_CheckSignals

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Temperature, Imu
from humanoid_league_msgs.msg import AdditionalServoData
from bitbots_speaker.msg import Speak
from bitbots_cm730.srv import SwitchMotorPower


import rospy

from .pose.pose import Joint, Pose
from .lowlevel.controller.controller import CM730, ID_CM730, MX28, BulkReadPacket, MultiMotorError, SyncWritePacket


class cm730_node(object):
    """
    Takes core of the communication with the CM730 (and therefore with the servos). Constantly updates servo data
    and sets goal values to the servos.
    """

    def __init__(self):
        rospy.init_node('bitbots_cm730', anonymous=False)
        rospy.Subscriber("/MotorGoals", JointTrajectory, self.update_motor_goals)
        self.joint_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.speak_publisher = rospy.Publisher('/speak', String, queue_size=10)
        self.temp_publisher = rospy.Publisher('/temperatur', Temperature, queue_size=10)
        self.imu_publisher = rospy.Publisher('/imu', Imu, queue_size=10)
        self.motor_power_service = rospy.Service("switch_motor_power", SwitchMotorPower, self.switch_motor_power_service_call)

        self.cm_730_object = CM730()

        self.raw_accel = IntDataVector(0, 0, 0)
        self.raw_gyro = IntDataVector(0, 0, 0)
        self.button1 = 0
        self.button2 = 0

        joints = rospy.get_param("/joints")
        robot_type_name = rospy.get_param("/RobotTypeName")
        self.motors = rospy.get_param(robot_type_name + "/motors")

        #problem is, that the number of motors is not known at build time, so write them into params now
        for motor in joints:
            min_value = -180
            max_value = 180
            if 'max' in motor['limits']:
                max_value = motor['limits']['max']
            if 'min' in motor['limits']:
                min_value = motor['limits']['min']
            rospy.set_param(str(motor['name']), {'min': min_value, 'max': max_value})
        #start the endless loop
        self.update_forever()


    cdef update_motor_goals(self, msg):
        """ Callback for subscription on motorgoals topic.
        We can only handle the first point of a JointTrajectory :( """
        joints = msg.joint_names
        #we can handle only one position, no real trajectory
        positions = msg.points[0].position
        for joint in joints:
            if positions[joint] > joint.max:
                self.motor_goals[joint] = joint.max
                rospy.logwarn("Joint command over max. Joint: %s Position: %f", (joint, positions[joint]))
            elif positions[joint] < joint.min:
                self.motor_goals[joint] = joint.max
                rospy.logwarn("Joint command under min. Joint: %s Position: %f", (joint, positions[joint]))
            else:
                self.motor_goals[joint] = positions[joint]

    cpdef update_forever(self):
        """ Calls :func:`update_once` in an infite loop """
        cdef int iteration = 0, errors = 0
        cdef double duration_avg = 0, start = time.time()

        while True:
            self.update_once()

            # Signal check
            PyErr_CheckSignals()

            # Count to get the update frequency
            iteration += 1
            if iteration < 100:
                continue

            if duration_avg > 0:
                duration_avg = 0.5 * duration_avg + 0.5 * (time.time() - start)
            else:
                duration_avg = (time.time() - start)

            rospy.loginfo("Updates/Sec %f", iteration / duration_avg)
            iteration = 0
            start = time.time()

    cpdef update_once(self):
        """ Updates sensor data with :func:`update_sensor_data`, publishes the data and sends the motor commands.

            The sensordata from the last iteration will be provided as smoothed values in
            :attr:`smooth_accel` and :attr:`smooth_gyro`.
        """
        # get sensor data
        self.cm_730_object.update_sensor_data()

        #todo irgendwas damit tun
        # Farbwerte für den Kopf holen
        self.led_eye = self.ipc.get_eye_color().xyz
        self.led_head = self.ipc.get_forehead_color().xyz

        # Send Messages to ROS
        self.publish_joints()
        self.publish_additional_servo_data()
        self.publish_IMU()
        self.publish_buttons()

        # send new position to servos
        self.update_motor_goals()


    cpdef switch_motor_power(self, state):
        # wir machen nur etwas be änderungen des aktuellen statusses
        if not self.cm_370:
            # without the cm370 we cant switch the motor power
            return
        if state and not self.dxl_power:
            # anschalten
            rospy.loginfo("Switch dxl_power back on")
            self.ctrl.write_register(ID_CM730, CM730.dxl_power, 1)
            # wir warten einen Augenblick bis die Motoeren auch wirklich wieder
            # wieder an und gebootet sind
            time.sleep(0.3)
            self.set_motor_ram()
            self.dxl_power = True
        elif (not state) and self.dxl_power:
            # ausschalten
            rospy.loginfo("Switch off dxl_power")
            # das sleep hier ist nötig da es sonst zu fehlern in der
            # firmware der Motoren kommt!
            # Vermutete ursache:
            # Schreiben der ROM area der Register mit sofortigen
            # abschalten des Stromes führt auf den motoren einen
            # vollst#ndigen Reset durch!
            time.sleep(0.3) # WICHTIGE CODEZEILE! (siehe oben)
            self.ctrl.write_register(ID_CM730, CM730.dxl_power, 0)
            self.dxl_power = False

    def switch_motor_power_service_call(self, req):
        return self.switch_motor_power(req.power)


    cdef void send_joints(self):
        """
        Sends the Joint States to ROS
        """
        cdef object ids = []
        cdef object positions = []
        cdef object speeds = []
        cdef object loads = []
        cdef msg = JointState()
        for name, joint in self.robo_pose.joints:
            ids.append(name)
            positions.append(math.radians(joint.position))
            speeds.append(joint.load)
            loads.append(joint.load)
        msg.header.stamp = rospy.Time.now()
        msg.name = ids
        msg.position = positions
        msg.velocity = speeds
        msg.effort = loads
        self.joint_publisher.publish(msg)

    def publish_additional_servo_data(self):
        msg = AdditionalServoData()
        cdef object temperatures = []
        for temp in self.temps:
            ros_temp = Temperature()
            ros_temp.header.stamp = rospy.Time.now()
            ros_temp.temperature = temp
            ros_temp.variance = 0
            temperatures.append(ros_temp)
        msg.temperature = temperatures
        msg.voltage = self.voltages
        self.temp_publisher(msg)

    def publish_IMU(self):
        msg = Imu()
        #todo check if this is realy linear and not angular acceleration
        msg.linear_acceleration = self.raw_accel
        # todo conversation to quaternion :(
        msg.orientation = self.raw_gyro
        self.imu_publisher(msg)

    def publish_buttons(self):

    def say(self, text):
        msg = Speak()
        msg.text = text
        self.speak_publisher.publish(msg)
#!/usr/bin/env python
#-*- coding:utf-8 -*-
import time

import math
#from Cython.Includes.cpython.exc import PyErr_CheckSignals

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Temperature, Imu
from humanoid_league_msgs.msg import AdditionalServoData
from humanoid_league_msgs.msg import Speak
from bitbots_cm730.srv import SwitchMotorPower
from bitbots_speaker.speaker import speak

import rospy



cdef class cm730_node(object):
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

        self.goal_pose = Pose()

        self.cm_730_object = CM730()

        joints = rospy.get_param("/joints")

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


    cpdef update_motor_goals(self, object msg):
        """ Callback for subscription on motorgoals topic.
        We can only handle the first point of a JointTrajectory :( """
        motor_goals = []
        joints = msg.joint_names
        #we can handle only one position, no real trajectory
        positions = msg.points[0].position
        for joint in joints:
            if positions[joint] > joint.max:
                motor_goals[joint] = joint.max
                rospy.logwarn("Joint command over max. Joint: %s Position: %f", (joint, positions[joint]))
            elif positions[joint] < joint.min:
                motor_goals[joint] = joint.min
                rospy.logwarn("Joint command under min. Joint: %s Position: %f", (joint, positions[joint]))
            else:
                motor_goals[joint] = positions[joint]
        # todo something like this
        # self.goal_pose =

    cpdef update_forever(self):
        """ Calls :func:`update_once` in an infite loop """
        cdef int iteration = 0, errors = 0
        cdef double duration_avg = 0, start = time.time()

        while True:
            self.update_once()

            # Signal check
            #PyErr_CheckSignals() #todo is this necessary?

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
        robo_pose, gyro, accel, button1, button2= self.update_sensor_data()

        # send new position to servos
        self.cm_730_object.apply_goal_pose(self.goal_pose)

        #todo irgendwas damit tun
        # Farbwerte für den Kopf holen
        #self.led_eye = self.ipc.get_eye_color().xyz
        #self.led_head = self.ipc.get_forehead_color().xyz

        # Send Messages to ROS
        self.publish_joints(robo_pose)
        # todo self.publish_additional_servo_data()
        self.publish_IMU(gyro, accel)
        self.publish_buttons(button1, button2)

    cpdef update_sensor_data(self):
        robo_pose= Pose()
        # first get data
        result , cid_all_values = self.cm_730_object.sensor_data_read()
        if not result:
            return
        if result == -1:
            #this means the motion is stuck
            rospy.logerr("motion stuck!")
            speak("motion stuck")
            exit("Motion stuck")

        #todo get temps and voltages
        #todo parse pose
        # parse data
        button, gyro, accel = self.cm_730_object.parse_sensor_data(result, cid_all_values)
        if button == -1:
            #low voltage error
            speak("Warning: Low Voltage! System Exit!")
            rospy.logerr("SYSTEM EXIT: LOW VOLTAGE")
            raise SystemExit("SYSTEM EXIT: LOW VOLTAGE (%d V)" % gyro/10)

        raw_accel = accel - IntDataVector(512, 512, 512)
        raw_gyro = gyro - IntDataVector(512, 512, 512)

        if button is not None:
            button1 = button & 1
            button2 = (button & 2) >> 1

        return robo_pose, raw_gyro, raw_accel, button1, button2


    cpdef switch_motor_power_service_call(self, object req):
        return self.cm_730_object.switch_motor_power(req.power)


    cpdef publish_joints(self, Pose robo_pose):
        """
        Sends the Joint States to ROS
        """
        cdef object ids = []
        cdef object positions = []
        cdef object speeds = []
        cdef object loads = []
        cdef msg = JointState()
        for name, joint in robo_pose.joints:
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

    cpdef publish_additional_servo_data(self, list temps, list voltages):
        msg = AdditionalServoData()
        cdef object temperatures = []
        for temp in temps:
            ros_temp = Temperature()
            ros_temp.header.stamp = rospy.Time.now()
            ros_temp.temperature = temp
            ros_temp.variance = 0
            temperatures.append(ros_temp)
        msg.temperature = temperatures
        msg.voltage = voltages
        self.temp_publisher(msg)

    cpdef publish_IMU(self, double gyro, double accel):
        msg = Imu()
        #todo check if this is realy linear and not angular acceleration
        msg.linear_acceleration = accel
        # todo conversation to quaternion :(
        msg.orientation = gyro
        self.imu_publisher(msg)

    cpdef publish_buttons(self, bool button1, bool button2):
        #todo impelement
        return
#!/usr/bin/env python
#-*- coding:utf-8 -*-
import rospy
import time

from .lowlevel.controller.controller import BulkReadPacket, MultiMotorError, MX28, ID_CM730, CM730, SyncWritePacket
from .pose.pose import Joint, Pose


class CM730(object):

    def __init__(self):
        self.read_packet_stub = list()
        self.read_packet2 = BulkReadPacket()
        self.read_packet3_stub = list()
        self.init_read_packet()

        self.low_voltage_counter = 0
        self.last_io_success = 0


    cdef init_read_packet(self):
        """
        Initialise the :class:`BulkReadPacket` for communication with the motors

        Important: The motor in self.read_packet[i] has to be the same like in self.read_packet3[i], because
         while reading, single packages from 1 are inserted to 3.
        """
        for cid in self.motors:
            # if robot has this motor
            self.read_packet_stub.append((
                cid,
                (
                    MX28.present_position,
                    MX28.present_speed,
                    MX28.present_load,
                    MX28.present_voltage,
                    MX28.present_temperature
                )))
            self.read_packet3_stub.append((
                cid,
                (
                    MX28.present_position,
                )))
        if self.cm_370:
            self.read_packet_stub.append((
                ID_CM730,
                (
                    CM730.button,
                    CM730.padding31_37,
                    CM730.gyro,
                    CM730.accel,
                    CM730.voltage
                )))
            self.read_packet2.add(
                ID_CM730,
                (
                    CM730.button,
                    CM730.padding31_37,
                    CM730.gyro,
                    CM730.accel,
                    CM730.voltage
                ))
            self.read_packet3_stub.append((
                ID_CM730,
                (
                    CM730.gyro,
                    CM730.accel
                )))

        if len(self.read_packet_stub)!= len(self.read_packet3_stub):
            raise AssertionError("self.read_packet and self.read_packet3 have to be the same size")

    cpdef update_sensor_data(self):
        result , cid_all_Values = self.sensor_data_read()
        if not result:
            return
        if result == -1:
            #motion stuck
            rospy.logwarn("motion stuck!")
            self.say("motion stuck")
        self.last_io_success = time.time()

        button, gyro, accel = self.parse_sensor_data(result, cid_all_Values)

        self.sensor_all_cid += 1

        self.update_gyro_data(gyro, accel)

        if button is not None:
            self.button1 = button & 1
            self.button2 = (button & 2) >> 1

    cdef sensor_data_read(self):
        """
        This Method is part of update_sensor_data,
                it communicates with the CM730-Board and extract its answer to a directly readable format
        """
        cdef dict result
        cdef int say_error
        # Das all_data Flag wird dazu benutzt das dann mehr daten
        # (tmperatur etc) abgefragt werden. Außerdem werden dann daten
        # an das Debug gesendet
        cdef int cid_all_values = 0
        cdef BulkReadPacket read_packet
        if self.sensor_all_cid >= len(self.read_packet_stub):
            self.sensor_all_cid = 0
        try:
            if self.dxl_power:
                    read_packet = BulkReadPacket()
                    for i in range(self.sensor_all_cid - 1):
                        read_packet.add(self.read_packet3_stub[i][0],self.read_packet3_stub[i][1])
                    read_packet.add(self.read_packet_stub[self.sensor_all_cid][0],self.read_packet_stub[self.sensor_all_cid][1])
                    cid_all_values = self.read_packet_stub[self.sensor_all_cid][0]
                    for i in range(self.sensor_all_cid +1, len(self.read_packet_stub)):
                        read_packet.add(self.read_packet3_stub[i][0],self.read_packet3_stub[i][1])
                    result = self.ctrl.process(read_packet)
            else:
                result = self.ctrl.process(self.read_packet2)
        except IOError, e:
            rospy.logdebug("Reading error: %s", str(e))
            if self.last_io_success > 0 and time.time() - self.last_io_success > 2:
                #we tell that we are stuck
                return (-1, -1)
            elif not  self.last_io_success > 0:
                self.last_io_success = time.time() + 5
                # This looks strange but is on purpose:
                # If it doesn't get any data, it should stop at _sometime_
            return (None, None)

        except MultiMotorError as errors:
            is_ok = True
            for e in errors:
                say_error = True
                err = e.get_error()
                if (err >> 0 & 1) == 1: # Imput Voltage Error
                    pass # mostly bullshit, ignore
                if (err >> 1 & 1) == 1: # Angel Limit Error
                    is_ok = False
                if (err >> 2 & 1) == 1: # Overheating Error
                    is_ok = False
                if (err >> 3 & 1) == 1: # Range Error
                    is_ok = False
                if (err >> 4 & 1) == 1: # Checksum Error
                    is_ok = False
                if (err >> 5 & 1) == 1: # Overload Error
                    say_error = False
                    if e.get_motor() in self.last_overload and \
                      time.time() - 2 < self.last_overload[e.get_motor()]:
                        self.overload_count[e.get_motor()] += 1
                        if self.overload_count[e.get_motor()] > 60:
                            rospy.logwarn("Raise long holding overload error")
                            is_ok = False # will be forwared
                    else:
                        # resetten, der letzte ist schon ne weile her
                        self.overload_count[e.get_motor()] = 0
                        rospy.logwarn("Motor %d has a Overloaderror, "
                            % e.get_motor() + " ignoring 60 updates")
                    self.last_overload[e.get_motor()] = time.time()
                if (err >> 6 & 1) == 1: # Instruction Error
                    is_ok = False
                if (err >> 7 & 1) == 1: # Unused
                    is_ok = False
                if say_error:
                    rospy.logerr(err, "A Motor has had an error:")
            if not is_ok:
                # If not everything was handled, we want to forward it
                # leads to shuting down the node
                raise
            # If an error was ignored, we have to test if a packed arrived
            # If not, we have to cancel, otherwise a uncomplete package will be handled
            result = errors.get_packets()
        return result, cid_all_values


    cdef parse_sensor_data(self, object sensor_data, object cid_all_Values):
        """
        This Method is part of update_sensor_data,
                it takes the data which we just read from the CM370 Board and parse it into the right variables
        """
        cdef Pose pose = self.robo_pose

        cdef Joint joint
        cdef IntDataVector accel = None
        cdef IntDataVector gyro = None
        #cdef maxtmp = 0, maxcid = -1
        #cdef min_voltage = 1e10, max_voltage = 0
        cdef position = None, speed=None, load=None
        cdef voltage = None, temperature=None, button=None

        for cid, values in sensor_data.iteritems():
            if cid == ID_CM730:
                #this is a reponse package from cm730
                if not cid_all_Values == ID_CM730 and self.dxl_power:
                    #only IMU values
                    gyro, accel = values
                else:
                    #get all values
                    button, _, gyro, accel, voltage = values
                    rospy.loginfo("CM730.Voltage %d", voltage)
                    if voltage < 105:
                        rospy.logwarn("Low Voltage!!")
                    if voltage < 100:
                        self.low_voltage_counter += 1
                        if self.low_voltage_counter > 10:
                            # we delay the low voltag shutdown because sometimes the hardware is telling lies
                            self.speak_publisher.publish("Warning: Low Voltage! System Exit!")
                            rospy.logerr("SYSTEM EXIT: LOW VOLTAGE")
                            raise SystemExit("SYSTEM EXIT: LOW VOLTAGE (%d V)" % voltage/10)
                    else:
                        self.low_voltage_counter = 0
            else:
                joint = pose.get_joint_by_cid(cid)
                if not cid_all_Values == cid:
                    position = values[0]
                else:
                    position, speed, load, voltage, temperature = values
                    joint.set_load(load)

                position = position - self.joint_offsets[cid]
                joint.set_position(position)
                joint.set_load(load)
                joint.set_speed(speed)

                # Get aditional servo data, not everytime cause its not so important
                if cid_all_Values == cid:  # etwa alle halbe sekunde
                    #todo in returning ändern
                    debug.log("MX28.%d.Temperatur" % cid, temperature)
                    debug.log("MX28.%d.Voltage" % cid, voltage)

                if temperature > 60:
                    fmt = "Motor cid=%d has a temperature of %1.1f°C: EMERGENCY SHUT DOWN!"
                    rospy.logwarn(fmt % (cid, temperature))
                    raise SystemExit(fmt % (cid, temperature))

        return button, gyro, accel



    cdef update_gyro_data(self, object gyro, object accel):
        cdef double dt, t
        cdef CDataVector angles
        #cdef Vector3f accle
        if gyro is not None and accel is not None:
            t = time.time()
            dt = t - self.last_gyro_update_time
            self.last_gyro_update_time = t

            self.robo_accel = accel - IntDataVector(512, 512, 512)
            self.raw_gyro = gyro - IntDataVector(512, 512, 512)

            angles = calculate_robot_angles(deref(self.robo_accel.get_data_vector()))
            angles = self.gyro_kalman.get_angles_pvv(angles, gyro - IntDataVector(512, 512, 512), dt)
            self.robo_angle = wrap_data_vector(angles)
            self.robo_gyro = self.raw_gyro #self.gyro_kalman.get_rates_v()

            self.robot.update(self.robo_pose)
            new_angle = kinematic_robot_angle(self.robot, self.zmp_foot_phase)
            diff = (new_angle[0] - self.last_kinematic_robot_angle[0]) / dt, (new_angle[1] - self.last_kinematic_robot_angle[1]) / dt
            angles.set_x(new_angle[0])
            angles.set_y(new_angle[1])
            angles = self.kinematic_kalman.get_angles_vfv(angles, CDataVector(diff[0],diff[1], 0.0), dt)
            self.last_kinematic_robot_angle = new_angle
            self.kin_robo_angle = wrap_data_vector(angles)


            #print "robo accel %s, kinematic_angle %s, robAngle %s" % (self.robo_accel, self.kin_robo_angle, self.robo_angle)
            diff_angles = (self.robo_angle - self.kin_robo_angle)


    cdef set_motor_ram(self):
        """
        This method sets the values in the RAM of the motors, dependent on the values in the config.
        """
        if rospy.get_param('/cm730/setMXRam', False):
            rospy.loginfo("setting MX RAM")
            if self.cm_370:
                self.ctrl.write_register(ID_CM730, CM730.led_head, (255, 0, 0))
            for motor in self.motors:
                for conf in self.motor_ram_config:
                    self.ctrl.write_register(motor, MX28.get_register_by_name(conf),
                        self.motor_ram_config[conf])
            if self.cm_370:
                self.ctrl.write_register(ID_CM730, CM730.led_head, (0, 0, 0))
            rospy.loginfo("Setting RAM Finished")

    cpdef apply_goal_pose(self):
        cdef Pose pose = self.goal_pose
        cdef SyncWritePacket packet

        if pose is None:
            return

        # Hier werden die Augenfarben gesetzt.
        # Dabei kann in der Config angegeben werden ob die Augen bei Penalty
        # rot werden, und ob sie ansonsten überhaupt genutzt werden
        if self.cm_370:
            packet = SyncWritePacket((CM730.led_head, CM730.led_eye))
            #todo make this a service
            if self.state == STATE_PENALTY and rospy.get_param("/cm730/EyesPenalty", false):
                packet.add(ID_CM730, ((255, 0, 0), (0, 0, 0)))
            else:
                if rospy.get_param("/cm730/EyesOff", False):
                    packet.add(ID_CM730, ((0, 0, 0), (0, 0, 0)))
                else:
                    packet.add(ID_CM730, (self.led_head, self.led_eye))

            self.ctrl.process(packet)

        cdef Joint joint
        cdef Joint joint2
        cdef SyncWritePacket goal_packet = None
        cdef SyncWritePacket torque_packet = None
        cdef SyncWritePacket p_packet = None
        cdef SyncWritePacket i_packet = None
        cdef SyncWritePacket d_packet = None
        cdef int joint_value = 0

        #todo make this a service?
        if self.state != STATE_SOFT_OFF: #todo soft off existiert nicht mehr

            if not self.dxl_power:
                self.switch_motor_power(True)
                # Aktuallisieren der Pose, da die Motoren mit hoher
                # warscheinlichkeit anders liegen als beim ausstellen
                self.update_sensor_data()
                # hier abbrechen um Zuckungen zu vermeiden
                return

            goal_packet = SyncWritePacket((MX28.goal_position, MX28.moving_speed))
            for name, joint in pose.joints:
                if not joint.has_changed():
                    continue

                if joint.is_active():
                    joint_value = int(joint.get_goal()) + \
                        self.joint_offsets[joint.get_cid()]
                    goal_packet.add(joint.get_cid(),
                        (joint_value, joint.get_speed()))
                else:  # if joint.get_cid() != 30:
                    # Torque muss nur aus gemacht werden, beim setzen eines
                    # Goals geht es automatisch wieder auf 1
                    # Das Torque-Packet nur erstellen, wenn wir es benötigen
                    # 30 ist virtuell und braucht daher nicht gesetzt werden
                    if torque_packet is None:
                        torque_packet = SyncWritePacket((MX28.torque_enable,))

                    # Motor abschalten
                    torque_packet.add(joint.get_cid(), (0, ))

                if joint.get_p() != -1:
                    if p_packet is None:
                        p_packet = SyncWritePacket((MX28.p,))
                    p_packet.add(joint.get_cid(), (joint.get_p(), ))
                    #print "set p:", joint.get_p(), joint.get_cid()

                if joint.get_i() != -1:
                    if i_packet is None:
                        i_packet = SyncWritePacket((MX28.i,))
                    i_packet.add(joint.get_cid(), (joint.get_i(), ))
                    #print "set p:", joint.get_p(), joint.get_cid()

                if joint.get_d() != -1:
                    if d_packet is None:
                        d_packet = SyncWritePacket((MX28.d,))
                    d_packet.add(joint.get_cid(), (joint.get_d(), ))
                    #print "set p:", joint.get_p(), joint.get_cid()

                # changed-Property wieder auf false setzen.
                joint.reset()

            # Zielwerte setzen
            self.ctrl.process(goal_packet)
            if torque_packet is not None:
                # Motoren abschalten, wennn nötig.
                self.ctrl.process(torque_packet)
            if p_packet is not None:
                self.ctrl.process(p_packet)
            if i_packet is not None:
                self.ctrl.process(i_packet)
            if d_packet is not None:
                self.ctrl.process(d_packet)
        else:
            if self.dxl_power:
                self.switch_motor_power(False)

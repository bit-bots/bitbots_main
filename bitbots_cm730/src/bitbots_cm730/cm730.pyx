#-*- coding:utf-8 -*-
import rospy
import time

from bitbots_cm730.lowlevel.controller.controller cimport BulkReadPacket, SyncWritePacket, Controller
from bitbots_cm730.lowlevel.controller.controller import MultiMotorError, MX28_REGISTER, CM730_REGISTER
from bitbots_cm730.lowlevel.controller.controller cimport ID_CM730
from bitbots_cm730.lowlevel.serial cimport Serial

from bitbots_common.pose.pypose cimport PyJoint as Joint
from bitbots_common.util import Joints as JointManager


cdef class CM730(object):

    def __init__(self):
        device = rospy.get_param("cm730/device")
        try:
            serial = Serial(device)
        except IOError:
            rospy.logwarn("No connection to cm730 on " + device + " Will try other possibilities")
            i = 0
            sucessfull = False
            while i < 5:
                try:
                    serial = Serial("/dev/ttyUSB" + str(i))
                    sucessfull = True
                    rospy.loginfo("/dev/ttyUSB" + str(i) + " worked :D")
                    break
                except IOError:
                    rospy.loginfo("/dev/ttyUSB" + str(i) + " did not work")
                    i += 1

            if not sucessfull:
                rospy.logerr("No connection to cm730 on " + device + " Please check the connection and restart the CM730 node.")
                rospy.sleep(0.1) # to make sure message arrives
                exit("Exit because of missing connection to the CM730 board.")

        self.low_voltage_counter = 0
        self.last_io_success = 0
        self.last_overload = {}
        self.overload_count = {}

        self.button1 = 0
        self.button2 = 0

        self.robo_pose = Pose()
        self.dxl_power = False
        self.sensor_all_cid = 0
        self.raw_gyro = IntDataVector(0, 0, 0)
        self.robo_accel = IntDataVector(0, 0, 0)

        robot_type_name = rospy.get_param("robot_type_name")
        self.motors = rospy.get_param("cm730/" + robot_type_name + "/motors")
        self.motor_ram_config = rospy.get_param("mx28config/RAM")
        self.motor_rom_config = rospy.get_param("mx28config/ROM")
        offsets = rospy.get_param("offsets")
        self.joints = rospy.get_param("joints")
        self.eye_param = rospy.get_param("cm730/EyesOff", False)

        self.joint_offsets =  [0]
        # sort offsets to list with motor ids
        for i in range(len(self.joints)):
            self.joint_offsets.append(offsets[self.joints[i]['name']])

        self.ctrl = Controller(serial)
        self.dxl_power = self.ctrl.read_register(ID_CM730, CM730_REGISTER.dxl_power)
        self.read_packet_stub = list()
        self.read_packet2 = BulkReadPacket()
        self.read_packet3_stub = list()
        self.init_read_packet()
        cdef bool old_dxl_power = self.dxl_power
        self.switch_motor_power(True)

        if rospy.get_param("cm730/motor_test"):
            motors_ok = True
            for i in range(len(rospy.get_param("cm730/Minibot/motors"))):
                # range goes from 0 to number of joints -1 so we increment by one
                if not self.ctrl.ping(i+1):
                    rospy.logwarn("Motor " + str(i) + " did not respond to ping")
                    motors_ok = False
            if motors_ok:
                rospy.logwarn("All motors were found")

        if rospy.get_param("cm730/setMXRom"):
            rospy.loginfo("Set Motor ROM")
            romsettings = self.motor_rom_config
            for i in range(len(rospy.get_param("cm730/Minibot/motors"))):
                self.ctrl.write_register(i,MX28_REGISTER.led, 1)
                for conf in romsettings:
                    self.ctrl.write_register(i,MX28_REGISTER.get_register_by_name(conf),
                        romsettings[conf])
                self.ctrl.write_register(i,MX28_REGISTER.led, 0)
            rospy.loginfo("Rom der Motoren gesetzt")

        self.switch_motor_power(old_dxl_power)

    cpdef init_read_packet(self):
        """
        Initialise the :class:`BulkReadPacket` for communication with the motors

        Important: The motor in self.read_packet[i] has to be the same like in self.read_packet3[i], because
         while reading, single packages from 1 are inserted to 3.
        """

        # IMU, buttons and voltage
        self.read_packet_stub.append((
            ID_CM730,
            (
                CM730_REGISTER.button,
                CM730_REGISTER.padding31_37,
                CM730_REGISTER.gyro,
                CM730_REGISTER.accel,
                CM730_REGISTER.voltage
            )))
        self.read_packet2.add(
            ID_CM730,
            (
                CM730_REGISTER.button,
                CM730_REGISTER.padding31_37,
                CM730_REGISTER.gyro,
                CM730_REGISTER.accel,
                CM730_REGISTER.voltage
            ))
        self.read_packet3_stub.append((
            ID_CM730,
            (
                CM730_REGISTER.gyro,
                CM730_REGISTER.accel
            )))
        for cid in self.motors:
            # if robot has this motor
            self.read_packet_stub.append((
                cid,
                (
                    MX28_REGISTER.present_position,
                    MX28_REGISTER.present_speed,
                    MX28_REGISTER.present_load,
                    MX28_REGISTER.present_voltage,
                    MX28_REGISTER.present_temperature
                )))
            self.read_packet3_stub.append((
                cid,
                (
                    MX28_REGISTER.present_position,
                    MX28_REGISTER.present_speed,
                )))

        if len(self.read_packet_stub)!= len(self.read_packet3_stub):
            raise AssertionError("self.read_packet and self.read_packet3 have to be the same size")

    cpdef sensor_data_read(self):
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
        except IOError as e:
            rospy.logdebug_throttle(1, "Reading error: " + str(e))
            if self.last_io_success > 0 and rospy.get_time() - self.last_io_success > 2:
                #we tell that we are stuck
                return -1, -1
            elif not  self.last_io_success > 0:
                self.last_io_success = rospy.get_time() + 5
                # This looks strange but is on purpose:
                # If it doesn't get any data, it should stop at _sometime_
            return None, None

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
                      rospy.get_time() - 2 < self.last_overload[e.get_motor()]:
                        self.overload_count[e.get_motor()] += 1
                        if self.overload_count[e.get_motor()] > 60:
                            rospy.logwarn("Raise long holding overload error")
                            is_ok = False # will be forwared
                    else:
                        # reset, the last was a while ago
                        self.overload_count[e.get_motor()] = 0
                        rospy.logwarn("Motor %d has a Overloaderror, "
                            % e.get_motor() + " ignoring 60 updates")
                    self.last_overload[e.get_motor()] = rospy.get_time()
                if (err >> 6 & 1) == 1: # Instruction Error
                    is_ok = False
                if (err >> 7 & 1) == 1: # Unused
                    is_ok = False
                if say_error:
                    rospy.logerr(err, "A Motor has had an error:")
            if not is_ok:
                # If not everything was handled, we want to forward it
                # leads to shutting down the node
                raise
            # If an error was ignored, we have to test if a packed arrived
            # If not, we have to cancel, otherwise a uncomplete package will be handled
            result = errors.get_packets()
        self.last_io_success = rospy.get_time()
        return result, cid_all_values


    cpdef parse_sensor_data(self, object sensor_data, int cid_all_values):
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
        cdef float position = 0, speed=0, load=0
        cdef float voltage = 0, temperature=0
        cdef button=None

        for cid, values in sensor_data.items():
            if cid == ID_CM730:
                #this is a reponse package from cm730
                if not cid_all_values == ID_CM730:#Todo: warum: and self.dxl_power:
                    #only IMU values
                    #but sometimes something is strange so make another test
                    if len(values) == 2:
                        gyro, accel = values
                    else:
                        button, _, gyro, accel, voltage = values
                else:
                    #get all values
                    button, _, gyro, accel, voltage = values
                    #rospy.logdebug_throttle(10, "CM730.Voltage: " + str(voltage))
                    if voltage < 105:
                        rospy.logwarn_throttle(1, "Low Voltage!!")
                    if voltage < 100:
                        self.low_voltage_counter += 1
                        if self.low_voltage_counter > 10:
                            # we delay the low voltag shutdown because sometimes the hardware is telling lies
                            return -1, voltage, -1, None
                    else:
                        self.low_voltage_counter = 0
            else:
                joint = pose.get_joint_by_cid(cid)
                if not cid_all_values == cid:
                    position, speed = values
                    joint.set_speed(speed)
                else:
                    position, speed, load, voltage, temperature = values
                    joint.set_load(load)
                    joint.set_speed(speed)

                position = position - self.joint_offsets[cid]
                joint.set_position(position)

                # Get aditional servo data, not everytime cause its not so important
                if cid_all_values == cid:  # etwa alle halbe sekunde
                    joint.set_temperature(temperature)
                    joint.set_voltage(voltage)

                if temperature > 60:
                    fmt = "Motor cid=%d has a temperature of %1.1f°C: EMERGENCY SHUT DOWN!"
                    rospy.logwarn(1, fmt % (cid, temperature))
                    raise SystemExit(fmt % (cid, temperature))
        self.sensor_all_cid += 1
        return button, gyro, accel, pose


    cpdef set_motor_ram(self):
        """
        This method sets the values in the RAM of the motors, dependent on the values in the config.
        """
        if rospy.get_param('cm730/setMXRam', False):
            rospy.loginfo("setting MX RAM")
            self.ctrl.write_register(ID_CM730, CM730_REGISTER.led_head, (255, 0, 0))
            for motor in self.motors:
                for conf in self.motor_ram_config:
                    self.ctrl.write_register(motor, MX28_REGISTER.get_register_by_name(conf),
                        self.motor_ram_config[conf])
            self.ctrl.write_register(ID_CM730, CM730_REGISTER.led_head, (0, 0, 0))
            rospy.loginfo("Setting RAM Finished")

    cpdef apply_goal_pose(self, object goal_pose):
        cdef Pose pose = goal_pose
        cdef SyncWritePacket packet

        if pose is None:
            return

        # Hier werden die Augenfarben gesetzt.
        # Dabei kann in der Config angegeben werden ob die Augen bei Penalty
        # rot werden, und ob sie ansonsten überhaupt genutzt werden
        packet = SyncWritePacket((CM730_REGISTER.led_head, CM730_REGISTER.led_eye))
        #todo make this a service
        #if self.state == STATE_PENALTY and rospy.get_param("/cm730/EyesPenalty", false):
        #    packet.add(ID_CM730, ((255, 0, 0), (0, 0, 0)))
        #else:
        if self.eye_param:
            packet.add(ID_CM730, ((0, 0, 0), (0, 0, 0)))
        else:
            #todo this looks like it even didnt work before
            #packet.add(ID_CM730, (self.led_head, self.led_eye))
            pass

        self.ctrl.process(packet)

        cdef Joint joint
        cdef Joint joint2
        cdef SyncWritePacket goal_packet = None
        cdef SyncWritePacket torque_packet = None
        cdef SyncWritePacket p_packet = None
        cdef SyncWritePacket i_packet = None
        cdef SyncWritePacket d_packet = None
        cdef int joint_value = 0

        # TODO: das ist mist hier, dann kan man den roboter nicht aus machen...
        #if not self.dxl_power:
        #    self.switch_motor_power(True)
        #    # We do nothing, so the actual pose gets updated before acting
        #    return

        goal_packet = SyncWritePacket((MX28_REGISTER.goal_position, MX28_REGISTER.moving_speed))
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
                    torque_packet = SyncWritePacket((MX28_REGISTER.torque_enable,))

                # Motor abschalten
                torque_packet.add(joint.get_cid(), (0, ))

            if joint.get_p() != -1:
                if p_packet is None:
                    p_packet = SyncWritePacket((MX28_REGISTER.p,))
                p_packet.add(joint.get_cid(), (joint.get_p(), ))
                #print "set p:", joint.get_p(), joint.get_cid()

            if joint.get_i() != -1:
                if i_packet is None:
                    i_packet = SyncWritePacket((MX28_REGISTER.i,))
                i_packet.add(joint.get_cid(), (joint.get_i(), ))
                #print "set p:", joint.get_p(), joint.get_cid()

            if joint.get_d() != -1:
                if d_packet is None:
                    d_packet = SyncWritePacket((MX28_REGISTER.d,))
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

    cpdef switch_motor_power(self, bool state):
        # wir machen nur etwas be änderungen des aktuellen statusses
        if state and not self.dxl_power:
            # anschalten
            rospy.loginfo("Switch dxl_power on")
            self.ctrl.write_register(ID_CM730, CM730_REGISTER.dxl_power, 1)
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
            # vollständigen Reset durch!
            time.sleep(0.3) # WICHTIGE CODEZEILE! (siehe oben)
            self.ctrl.write_register(ID_CM730, CM730_REGISTER.dxl_power, 0)
            self.dxl_power = False
        return True
#!/usr/bin/env python
#-*- coding:utf-8 -*-
import time
from std_msgs.msg import String

from sensor_msgs.msg import JointState

import rospy

class cm730(object):
    """
    update forever updated alle hardware daten und published die. schreibt local gespeicherte motorgoals
    wenn motor goals kommen werden die local gespeichert und dann von update_forever geschrieben, wenn der zyklus da angekommen ist
    """

    def __init__(self):
        rospy.init_node('bitbots_cm730', anonymous=False)
        rospy.Subscriber("/MotorGoals", JointTrajectory, self.update_motor_goals)
        self.joint_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.speak_publisher = rospy.Publisher('/speak', String, queue_size=10)
        self.temp_publisher = rospy.Publisher('/temperatur', Temperature, queue_size=10)

        self.raw_gyro = IntDataVector(0, 0, 0)
        self.smooth_accel = DataVector(0, 0, 0)
        self.smooth_gyro = DataVector(0, 0, 0)

        joints =  rospy.get_param("/joints")
        robot_type_name = rospy.get_param("/RobotTypeName")
        self.motors = rospy.get_param(robot_type_name + "/motors")

        self.init_read_packet()

        #todo make min/max dynamically reconfigurable
        #problem is, that the number of motors is not known at build time
        rospy.loginfo("Set Motor min/max Values")
        for motor in joints:
            min_value = -180
            max_value = 180
            if 'max' in motor['limits']:
                max_value = motor['limits']['max']
            if 'min' in motor['limits']:
                min_value = motor['limits']['min']
            rospy.set_param(str(motor['name']), {'min': min_value, 'max': max_value})

        #todo testen ob werte die man in den motor setzt in den maximas liegen

        self.update_forever()


    def update_motor_goals(self):
        """ Callback for subscription on motorgoals topic """
        #todo save motor values locally to get them updated during the update_forever loop
        #todo irgendwie das handeln, falls die motion falsche winkel sendet (außerhalb von maxima)


    cdef set_motor_ram(self):
        """
        Diese Methode setzt die Werte im Ram der Motoren, je nach wert aus
        der Configdatei
        """
        if rospy.get_param('/cm730/setMXRam', False):
            rospy.loginfo("setze MX28 RAM")
            if self.cm_370:
                self.ctrl.write_register(ID_CM730, CM730.led_head, (255, 0, 0))
            for motor in self.motors:
                for conf in self.motor_ram_config:
                    self.ctrl.write_register(motor, MX28.get_register_by_name(conf),
                        self.motor_ram_config[conf])
            if self.cm_370:
                self.ctrl.write_register(ID_CM730, CM730.led_head, (0, 0, 0))
            rospy.loginfo("Setze Ram Fertig")

    cdef init_read_packet(self):
        """
        Initiallisiert die :class:`BulkReadPacket` für die Kommunikation mit
        den Motoren

        Wichtig: Der Motor in self.read_packet[i] muss gleich dem self.read_packet3[i] sein, da beim Lesen einzelne
        packete aus 1 in 3 eingefügt werden.
        """
        self.read_packet_stub = list()
        self.read_packet2 = BulkReadPacket()
        self.read_packet3_stub = list()
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
            raise AssertionError("self.read_packet und self.read_packet3 müssen gleich lang sein")

    cpdef update_forever(self):
        """ Calls :func:`update_once` in an infite loop """
        cdef int iteration = 0, errors = 0
        cdef double duration_avg = 0, start = time.time()

        while True:
            self.update_once()

            # Signale prüfen
            PyErr_CheckSignals()

            # Mitzählen, damit wir eine ungefäre Update-Frequenz bekommen
            iteration += 1
            if iteration < 100:
                continue

            if duration_avg > 0:
                duration_avg = 0.5 * duration_avg + 0.5 * (time.time() - start)
            else:
                duration_avg = (time.time() - start)

            rospy.loginfo("Updates/Sec %d", iteration / duration_avg)
            iteration = 0
            start = time.time()

    cpdef update_once(self):
        """ Updates sensor data with :func:`update_sensor_data`, publishes the data and sends the motor commands.

            Die Sensordaten der letzten Iterationen werden in
            einem geglätteten Mittelwert als :attr:`smooth_accel` und
            :attr:`smooth_gyro` zur Verfügung gestellt.
        """
        # get sensor data
        self.update_sensor_data()


        # Gyrowerte geglättet merken
        #wird zur Fallerkennung genutzt, smooth_gyro ist dafür zu spät aber peaks muessen trotzdem geglaettet werden
        #glättung erhöhen -->  spätere erkennung
        #glättung verringern --> häufigeres Fehlverhalten, zb.: auslösen des Fallens beim Laufen
        self.smooth_gyro = self.smooth_gyro * 0.9 + self.raw_gyro * 0.1 ###gyro
        self.smooth_accel = self.smooth_accel * 0.9 + self.robo_accel * 0.1 ###accel


        # Send Joint Messages to Ros
        self.publish_joints()
        self.publish_temperatures()
        self.publish_raw_IMU()
        self.publish_smooth_IMU()
        self.publish_buttons()

        #todo motorwerte setzen

    cpdef update_sensor_data(self):
        result , cid_all_Values = self.sensor_data_read()
        if not result:
            return
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
                it communicates withe the CM370-Board and extract its answer to a directly readble format
        """
        cdef dict result
        cdef int say_error
        # Das all_data Flag wird dazu benutzt das dann mehr daten
        # (tmperatur etc) abgefragt werden. Außerdem werden dann daten
        # an das Debug gesendet
        cdef int cid_all_Values = 0
        cdef BulkReadPacket read_packet
        if self.sensor_all_cid >= len(self.read_packet_stub):
            self.sensor_all_cid = 0
        try:
            if self.dxl_power:
                    read_packet = BulkReadPacket()
                    for i in range(self.sensor_all_cid - 1):
                        read_packet.add(self.read_packet3_stub[i][0],self.read_packet3_stub[i][1])
                    read_packet.add(self.read_packet_stub[self.sensor_all_cid][0],self.read_packet_stub[self.sensor_all_cid][1])
                    cid_all_Values = self.read_packet_stub[self.sensor_all_cid][0]
                    for i in range(self.sensor_all_cid +1, len(self.read_packet_stub)):
                        read_packet.add(self.read_packet3_stub[i][0],self.read_packet3_stub[i][1])
                    result = self.ctrl.process(read_packet)
            else:
                result = self.ctrl.process(self.read_packet2)
        except IOError, e:
            rospy.logdebug("Lesefehler: %s", str(e))
            if self.last_io_success > 0 and time.time() - self.last_io_success > 2:
                rospy.logwarn("Motion hängt!")
                raise SystemExit("Motion hängt!")
            elif not  self.last_io_success > 0:
                self.last_io_success = time.time() + 5
                # Das sieht komisch aus, ist aber absicht:
                # Wenn er nimals daten bekommt soll er _irgentwann_ auch
                # aufhören es zu versuchen
            return (None, None)

        except MultiMotorError as errors:
            is_ok = True
            for e in errors:
                say_error = True
                err = e.get_error()
                if (err >> 0 & 1) == 1: # Imput Voltage Error
                    pass # die sind meist sowieso mist
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
                            is_ok = False # dann wirds jetzt weitergegen
                    else:
                        # resetten, der letzte ist schon ne weile her
                        self.overload_count[e.get_motor()] = 0
                        rospy.logwarn("Motor %d hat einen Overloaderror, "
                            % e.get_motor() + " ignoring 60 updates")
                    self.last_overload[e.get_motor()] = time.time()
                if (err >> 6 & 1) == 1: # Instruction Error
                    is_ok = False
                if (err >> 7 & 1) == 1: # Unused
                    is_ok = False
                if say_error:
                    rospy.logerr(err, "A Motor has had an error:")
            if not is_ok:
                # wenn es nicht alles behandelt wurde wollen wir es
                # weiterreichen (führt zum beenden der Motion)
                raise
            # wenn der fehler ignoriert wurde müssen wir prüfen ob ein packet
            # angekommen ist, wenn nicht abbrechen da
            # sonst ein unvollständiges packet verarbeitet wird
            result = errors.get_packets()
        return result, cid_all_Values


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
                if not cid_all_Values == ID_CM730 and self.dxl_power:
                    gyro, accel = values
                else:
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

                # Debug Informationen senden (nur alle 40, wegen der Datenmenge)
                if cid_all_Values == cid:  # etwa alle halbe sekunde
                    #todo in publishing ändern
                    debug.log("MX28.%d.Temperatur" % cid, temperature)
                    debug.log("MX28.%d.Voltage" % cid, voltage)
                    debug.log("MX28.%d.Load" % cid, load)

                if temperature > 60:
                    fmt = "Motor cid=%d hat eine Temperatur von %1.1f°C: Notabschaltung!"
                    rospy.logwarn(fmt % (cid, temperature))
                    self.speak_publisher.publish("An Motor has a Temperatur over %.1f°C" % temperature)
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

            # Das ist veraltet (zumindest momentan @Nils 30.3.15
            #if self.config["RobotTypeName"] == "Hambot":
            #    # Goal hat den Gyro verdreht eingebaut
            #    accel = IntDataVector(accel.y, 1024 - accel.z, 1024 - accel.x)
            #    gyro = IntDataVector(gyro.z, gyro.x, gyro.y)

            self.robo_accel = accel - IntDataVector(512, 512, 512)
            #print "Accel ungedreht %s" % self.robo_accel
            #self.robo_accel = verdrehe_bla(self.robo_accel)
            #print "Accel gedreht %s" % self.robo_accel
            self.raw_gyro = gyro - IntDataVector(512, 512, 512)

            angles = calculate_robot_angles(deref(self.robo_accel.get_data_vector()))
            #TODO Das verdrehe bla ist teil eines Features um Hambot momentan verdrehen Gyro zu fixen
            #angles = self.gyro_kalman.get_angles_pvv(angles, verdrehe_bla(gyro - IntDataVector(512, 512, 512)), dt)
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

        if self.state != STATE_SOFT_OFF:

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




    cdef void send_joints_ros(self):
        """
        Sends the Joint States to ROS
        """
        cdef object ids = []
        cdef object values = []
        cdef packet = JointState()
        for name, joint in self.robo_pose.joints:
            ids.append(name)
            values.append(math.radians(joint.position))
        packet.header.stamp = rospy.Time.now()
        packet.name = ids
        packet.position = values
        self.joint_publisher.publish(packet)


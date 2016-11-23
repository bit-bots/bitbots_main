# -*- coding: utf8 -*-
import rospy
import time

cdef state_to_string(int state):
    return {
        STATE_CONTROLABLE: "controlable",
        STATE_FALLING: "falling",
        STATE_FALLEN: "fallen",
        STATE_GETTING_UP: "getting up",
        STATE_ANIMATION_RUNNING: "animation running",
        STATE_BUSY: "busy",
        STATE_STARTUP: "startup",
        STATE_PENALTY: "penalty",
        STATE_PENALTY_ANIMANTION: "penalty animation",
        STATE_RECORD: "recording",
        STATE_SOFT_OFF: "soft off",
        STATE_WALKING: "walking",
        STATE_GETTING_UP_Second: "getting up2",
    }.get(state, "unknown state %d" % state)

MOVING_STATES = (STATE_CONTROLABLE,
                    STATE_WALKING,
                    STATE_RECORD,
                    STATE_SOFT_OFF)


cdef class Motion(object):

    def __init__(self):
        rospy.init_node('bitbots_motion', anonymous=False)
        rospy.Subscriber("/IMU", JointState, self.update_imu)
        rospy.Subscriber("/MotorCurrentPosition", JointState, self.update_current_position)
        rospy.Subscriber("/Buttons", Buttons, self.update_buttons)
        rospy.Subscriber("/Gamestate", Gamestate, self.update_gamestate)
        self.joint_goal_publisher = rospy.Publisher('/MotorGoals', JointState, queue_size=10)
        self.state_publisher = rospy.Publisher('/MotionState', JointState, queue_size=10)
        self.speak_publisher = rospy.Publisher('/speak', String, queue_size=10)

        self.accel = DataVector(0,0,0)
        self.gyro = DataVector(0,0,0)
        self.motor_current_position =
        self.button1 = 0
        self.buttons2 = 0
        self.gamestate =
        self.state = STARTUP #todo über launch startwert setzen

        self.standupHandler = StandupHandler()
        self.standupHandler.load_falling_data(self.config)

        if self.state == STATE_RECORD: #todo check if stil needed
            rospy.logwarn("Recordflag gesetzt, bleibe im record modus")
            rospy.logwarn("Wenn das falsch gesetzt ist, starte einmal das record script und beende es wieder")
            self.set_state(STATE_RECORD)
        else:
            self.set_state(STATE_STARTUP)

        if softstart and not self.state == STATE_RECORD:
            rospy.loginfo ("Softstart")
            # time -120 damit softoff wieder anstellen nicht anspringt
            self.last_client_update = time.time() - 120
            self.last_version = self.ipc.get_version()
            # das muss hgier schon, sonst geht die src sofort aus,
            # weil er merkt das noch kein update gekommen ist
            self.set_state(STATE_SOFT_OFF)
            self.switch_motor_power(False)
        else:
            self.last_client_update = time.time()
            self.switch_motor_power(True)

    def update_imu(self):

    def update_current_position(self):

    def update_buttons(self):

    def update_gamestate(self):

    def publish_motion_state(self):


    cpdef set_state(self, int state):
        """ Updatet den internen Status des MotionServers und publiziert
            ihn zum Clienten
        """
        #Wenn die Motion aus einem State kommt, in dem sie Aufsteht soll der Gyro resettet werden
        #todo kalman scheint nicht mehr benutzt zu werden, löschen?
        if self.state in [
                STATE_FALLEN,
                STATE_FALLING,
                STATE_GETTING_UP,
                STATE_PENALTY,
                STATE_PENALTY_ANIMANTION,
                STATE_STARTUP,
                STATE_BUSY,
                STATE_GETTING_UP_Second]:
            self.gyro_kalman.reset()
        self.state = state
        # unterdrücke state_soft_off nach außen, das sich clients noch trauen die src anzusprechen
        self.publish_motion_state(state if state != STATE_SOFT_OFF else STATE_CONTROLABLE)

        rospy.loginfo("Setze Status auf '%s'" % state_to_string(state))
        rospy.loginfo("Status", state_to_string(state))

    cpdef update_forever(self):
        """ Ruft :func:`update_once` in einer Endlosschleife auf """
        cdef int iteration = 0, errors = 0
        cdef double duration_avg = 0, start = time.time()

        while True:
            self.update_once()

            # im Softoff passiert nichts geschwindigkeit Kritisches, da brauchen
            # wir nicht ständig alles aktuallisieren, entlastet den cpu
            if self.state == STATE_SOFT_OFF:
                time.sleep(0.05)

            # Mitzählen, damit wir eine ungefäre Update-Frequenz bekommen
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

        #Warten, bis Startup erreicht ist.
        if self.startup_time is None or time.time() - self.startup_time > 1:
            if self.startup_time is not None:
                # Wenn wir das erste mal starten, alles initialisieren
                self.startup_time = None
                self.last_version = self.ipc.get_version()
                self.goal_pose.goals = self.robo_pose.positions
                if self.state != STATE_RECORD:
                    #wenn wir in record sind wollen wir da auch bleiben
                    self.ipc.set_state(STATE_CONTROLABLE)

            # wenn der Client neu in Penalized ist, setzen wir uns noch hin
            # sonst wollen wir nichts machen
            if self.ipc.get_state() == STATE_PENALTY:
                # Wenn der Roboter schon penalized ist, hören wir hier auf.
                if self.state == STATE_PENALTY:
                    # Motoren Abstellen, sind nicht nötig, um das interne
                    # handling kümmert sich die implementation (sensor_update
                    # wird weiterhin aufgeruffen)
                    self.switch_motor_power(False)
                    time.sleep(0.05)
                    # wir tuen so als wenn es noch updates vom client gibt
                    # um dafür zu sorgen das es nach dem aufwachen vom
                    # penalty nicht sofort nach softoff oder off aufgrund
                    # der lange zeit (genzwungenen) keine updates geht
                    # (in STATE_PENALTY ist es unmöglich updates and die
                    # notion zu liefern)
                    self.last_client_update = time.time()
                    return
                # sonst hinsetzen, aber nur wenn wir das nicht sowieso schon tun
                if self.state != STATE_PENALTY_ANIMANTION:
                    rospy.logwarn("Penalized, sit down!")
                    self.animate(
                        rospy.get_param("/animations/motion/penalized", STATE_PENALTY)
                    self.set_state(STATE_PENALTY_ANIMANTION)
                    rospy.logwarn("Motion wird danach nichts mehr tun, penalized")

            if self.state == STATE_PENALTY:
                # Die Motion ist noch im State Penalty, der Client nicht mehr,
                # wir stehen auf und stehen wieder auf und setzen uns 'steuerbar'.

                # Als erstes stellen wir die Motoren wieder an
                self.switch_motor_power(True)
                # Aktuelle Pose hohlen um Zuckungen zu vermeiden
                self.update_sensor_data()
                self.set_state(STATE_PENALTY_ANIMANTION)
                # Aufstehen
                self.animate("Automatische Walkready", STATE_CONTROLABLE, self.walkready_animation)

            if self.ipc.get_state() == STATE_RECORD:
                # wenn der Client im Record ist, wollen wir nicht "Controlabel"
                #sein da sonst andere Clienten die Steuerung übernehmen könnten
                if self.state == STATE_SOFT_OFF:
                    #wenn im softoff erstmal aufstehen (Ist besser)
                    self.animate(
                        rospy.get_param("/animations/motion/walkready", STATE_RECORD)
                elif self.state not in (
                        STATE_RECORD,
                        STATE_ANIMATION_RUNNING ,
                        STATE_GETTING_UP,
                        STATE_STARTUP,
                        STATE_GETTING_UP_Second):
                    # wenn wir gerade ne animation spielen gehen wir vermutlich
                    # gerade zum record
                    self.set_state(STATE_RECORD)

            elif self.state == STATE_RECORD:
                # nach abgeschlossener aufnahme wieder in den normalzustand
                # zurückkehren
                self.set_state(STATE_CONTROLABLE)

            # Farbwerte für den Kopf holen
            self.led_eye = self.ipc.get_eye_color().xyz
            self.led_head = self.ipc.get_forehead_color().xyz

            if self.state not in (STATE_RECORD, STATE_SOFT_OFF) and self.ipc.get_state() != STATE_ANIMATION_RUNNING:
                # nur evaluieren, wenn der Client keine Animation spielt und
                # nicht im Record Modus ist (den prüfen wir bei uns da wir
                # bevor wir endgültig nach record gehen unter umständen
                # noch eigene dinge tun, danach uns selber m it state
                # record sperren). Im SOFT_OFF können wir sowieso nicht aufstehen,
                # und wenn man da drinn eine animation setzt kann es zu Problemen kommen
                self.evaluate_state()

    cpdef evaluate_state(self):
        """ :func:`evaluate_state` ermittelt aus dem zuletzt in
            :func:`update_sensor_data` gespeicherten Zustand die nötigsten
            Aktionen wie *Aufstehen* oder die
            *Gelenke weichstellen*, wenn der Roboter umgefallen ist. Dafür
            ruft sie Funktionen wie :func:`check_queued_animations`
            oder :func:`checked_fallen` auf.
        """
        self.check_queued_animations()
        if self.standupflag:
            anim = self.standupHandler.check_fallen(self.goal_pose , self.set_state, self.state,
                              self.smooth_gyro, self.robo_angle, self.smooth_accel)
            if anim:
                self.animate(anim)


     cpdef update_goal_pose(self):
        """ Updated :attr:`goal_pose`. Wenn eine Animation läuft, spielt
            die mit hinein. Sonst kommt sie vermutlich vom Client. Außerdem
            werden hier alle anderen Befehle vom Client verarbeitet (walking,
            tracking)
        """
        cdef Pose pose
        cdef Joint joint
        cdef int version

        self.walk_forward = self.ipc.get_walking_forward()
        self.walk_sideward = self.ipc.get_walking_sidewards()
        self.walk_angular = self.ipc.get_walking_angular()
        self.walk_active = self.ipc.get_walking_activ()

        if self.ipc.get_reset_tracking():
            self.ipc.set_last_track(self.ipc.get_last_track())
            self.ipc.set_last_track_time(self.ipc.get_last_track_time())

        if self.animfunc is not None and self.state in (
                STATE_ANIMATION_RUNNING,
                STATE_GETTING_UP,
                STATE_GETTING_UP_Second,
                STATE_PENALTY_ANIMANTION,
                STATE_BUSY):
            # Wenn gerade eine Animation spielt, nehmen wir die
            # nächste Pose aus der Animation
            pose = self.animfunc(self.robo_pose)
            # wir reseten das walking
            self.walking.set_velocity(0, 0, 0)
            self.walking.stop()
            self.walking.set_active(False)

            if pose is not None:
                self.goal_pose.update(pose)
                return

            # Animation ist zuende
            rospy.loginfo("End of Animation")
            self.animfunc = None
            self.last_version = self.ipc.get_version()
            # TODO: Use config
            if self.state is STATE_GETTING_UP:
                anim = self.standupHandler.get_up(self.set_state)
                if anim:
                    self.animate(anim)
                if self.state is STATE_FALLEN:
                    anim = self.standupHandler.check_fallen(self.goal_pose , self.set_state, self.state,
                              self.smooth_gyro, self.robo_angle, self.smooth_accel)
                    if anim:
                        self.animate(anim)
            # Status setzen
            else:
                if self.post_anim_state is None:
                    self.set_state(STATE_CONTROLABLE)
                # Wir kommen vom Standup nach dem Hinfallen und wollen uns vor dem Laufen noch mal stabilisieren
                # spezieller endstatus gewünscht, setzen und wunsch auf null setzen
                else:
                    self.set_state(self.post_anim_state)
                    self.post_anim_state = None

        if self.state is STATE_GETTING_UP_Second:
            print self.walkready_animation
            self.animate("Automatische Walkready", STATE_CONTROLABLE , animation=self.walkready_animation)


        if self.walking.running:
            # Das Walking läuft aktuell
            if self.state == STATE_WALKING:
                # Wir Sind im richtigen status, und berechnen die
                # nächste Pose
                if not self.walk_active:
                    # wir laufen, wollen aber anhalten
                    self.walking_stop()
                    # Danach Trotzdem noch weiter die Pose
                    # berechnen, da der Schritt noch beendet werden
                    # sollte
                self.goal_pose.update(self.calculate_walking())
                self.last_client_update = time.time()
            else:
                # Durch irgendetwas sind wir nichtmehr im STATE_WALKING
                # das bedeutet das wir von der Motion gezwungen wurden
                # etwas anderes zu tun (z.B. Aufstehn/Hinfallen)
                # Dann reseten Wir das Walking und stoppen dabei.
                self.walking_reset()
                p = rospy.get_param("/mx28config/RAM/p")
                i = rospy.get_param("/mx28config/RAM/i")
                d = rospy.get_param("/mx28config/RAM/d")
                for name, joint in self.goal_pose.joints:
                    joint.p = p
                    joint.i = i
                    joint.d = d
        else:  # if self.walking.running:
            # Walking läuft nicht, prüfen opb wir laufen sollen
            if self.walk_active:
                if self.ipc.get_state() in (
                            STATE_CONTROLABLE,
                            STATE_SOFT_OFF,
                            STATE_WALKING
                        ) and self.state in (
                            STATE_CONTROLABLE,
                            STATE_SOFT_OFF,
                            STATE_WALKING
                        ):
                    # Wir machen das Walking an wenn der Client entweder
                    # Controlabel oder in Soft_OFF sind.
                    # STATE_WALKING steht hier mit falls irgendwo etwas
                    # schiefläuft damit wir nicht bei inkonsistenten
                    # zuständen nicht mehr laufen können
                    self.set_state(STATE_WALKING)
                    self.walking_started = time.time()
                    self.walking_start()
            elif self.state == STATE_WALKING:
                # Das Walking ist gestoppt, und wir sollen auch nicht
                # laufen, es ist aber noch STATE_WALKING, dann sind
                # wir gerade erfolgreich angehalten und müssen den Status
                # noch wieder ändern:
                self.set_state(STATE_CONTROLABLE)
                # wir setzen das p value aller motoren neu, da das
                # walking hier rumspielt, es für das meiste andere
                # (auch stehen) nicht gut ist wenn die reduziert sind
                p = rospy.get_param("/mx28config/RAM/p")
                i = rospy.get_param("/mx28config/RAM/i")
                d = rospy.get_param("/mx28config/RAM/d")
                for name, joint in self.goal_pose.joints:
                    joint.p = p
                    joint.i = i
                    joint.d = d

        with self.ipc:
            # Möglicherweise gibt es eine neue Pose vom Client, dann nehmen
            # wir diese, Wenn wir Controlabel sind
            now = time.time()
            version = self.ipc.get_version()

            """
            if version > self.last_version and self.state in (
                    MOVING_STATES):
                self.last_client_update = now
                self.last_version = version

                # Pose aus dem IPC holen, interne Kopie updaten und
                # Änderungen im IPC vergessen.
                pose = self.ipc.get_pose_ref()
                self.goal_pose.update(pose)
                pose.reset()
            """

        if self.dieflag and now - self.last_client_update > 60 and \
            not self.state in (
                    STATE_RECORD,
                    STATE_ANIMATION_RUNNING,
                    STATE_WALKING,
                    STATE_PENALTY_ANIMANTION,
                    STATE_PENALTY):
            # es gab in den letzten 60 Sekunden keine Aktuallisierung
            # der Pose vom Client, wir stellen uns hier dann mal aus
            # wenn wir nicht im STATE_RECORD sind, da ist das meistens
            # eher blöd wenn wir einfach ausgehen, wenn wir in
            # STATE_ANIMATION_RUNNING sind, tuen wir selbst offensichtlich
            # etwas, z.B. wieder Aufstehen. Wenn wir im State Walking sind
            # tuen wir auch etwas gerade, beim beenden vom Walking gibt
            # es ein update
            if self.softoff:
                if self.state != STATE_SOFT_OFF:
                    # wir sind schon im Soft-Off
                    rospy.logwarn("Gehe in soft-off status")
                    say("Switch to soft-off")
                    if self.state != STATE_CONTROLABLE:
                        # Wenn wir nicht Controllabel sind können wir uns
                        # nicht hinsetzen, macht dann nicht so viel sinn
                        # es zu versuchen.
                        self.set_state(STATE_SOFT_OFF)
                    else:
                        # Wir setzen hier auf Busy da es sonst passieren
                        # könnte das die Animation nicht abgespielt wird
                        self.set_state(STATE_BUSY)
                        self.animate(rospy.get_param("/animations/motion/sit-down", STATE_SOFT_OFF)
            else:
                # wenn wir hierher kommen und softoff sind, dann ist es wohl
                # ein softstart ohne softoff, also ok
                if self.state != STATE_SOFT_OFF:
                    # es soll nur aus gemacht werden
                    rospy.logwarn("Kein Update vom Client in 60 Sekunden")
                    self.speak_publisher.publish("No update in the last 60 Seconds")
                    rospy.loginfo("Bye")
                    raise SystemExit()
        elif self.state == STATE_SOFT_OFF:
            # Pose resetten um versehentliche sprünge zu vermeiden
            # wir wollen jetzt erstmal aufstehen
            self.goal_pose.reset()
            rospy.loginfo("Update nach Softoff eingetroffen reaktivieren")
            self.set_state(STATE_STARTUP)

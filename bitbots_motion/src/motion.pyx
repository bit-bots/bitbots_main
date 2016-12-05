# -*- coding: utf8 -*-
from std_msgs.msg import String

from sensor_msgs.msg import JointState
from bitbots_common.utilCython.pydatavector cimport PyDataVector as DataVector

import rospy
import time

from .standuphandler cimport StandupHandler
from bitbots_cm730.srv import SwitchMotorPower

from dynamic_reconfigure.server import Server
from .cfg import bitbots_motion_params


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

    def __init__(self, dieflag, standupflag, softoff, softstart):
        rospy.init_node('bitbots_motion', anonymous=False)
        rospy.Subscriber("/IMU", JointState, self.update_imu)
        rospy.Subscriber("/MotorCurrentPosition", JointState, self.update_current_position)
        rospy.Subscriber("/Buttons", Buttons, self.update_buttons)
        rospy.Subscriber("/Gamestate", GameState, self.update_gamestate)
        self.joint_goal_publisher = rospy.Publisher('/MotorGoals', JointState, queue_size=10)
        self.state_publisher = rospy.Publisher('/MotionState', JointState, queue_size=10)
        self.speak_publisher = rospy.Publisher('/Speak', String, queue_size=10)
        self.dyn_reconf = Server(bitbots_motion_params, self.reconfigure)

        self.dieflag = dieflag
        self.softoff = softoff
        self.standupflag = standupflag
        self.softstart = softstart

        self.accel = DataVector(0,0,0)
        self.gyro = DataVector(0,0,0)
        self.smooth_accel = DataVector(0, 0, 0)
        self.smooth_gyro = DataVector(0, 0, 0)
        self.not_much_smoothed_gyro = DataVector(0, 0, 0)

        #todo chek what it is and if it is used rightly
        self.robo_angle = DataVector(0, 0, 0)

        self.motor_current_position =
        self.motor_goal_position =
        self.startup_time = time.time()
        self.first_run = True

        self.gamestate =
        self.penalize_active = False # penalized from game controller
        self.record_active = False # record UI in use
        self.animation_requested = False # animation request from animation server

        self.state = None
        self.set_state(STATE_STARTUP)
        self.stand_up_handler = StandupHandler()


        if softstart and not self.state == STATE_RECORD:
            rospy.loginfo ("Softstart")
            # time -120 damit softoff wieder anstellen nicht anspringt
            self.last_client_update = time.time() - 120
            # das muss hgier schon, sonst geht die src sofort aus,
            # weil er merkt das noch kein update gekommen ist
            self.set_state(STATE_SOFT_OFF)
            self.switch_motor_power(False)
        else:
            self.last_client_update = time.time()
            self.switch_motor_power(True)

    def switch_motor_power(self, state):
        """ Calling service from CM730 to turn motor power on or off"""
        rospy.wait_for_service("switch_motor_power")
        power_switch = rospy.ServiceProxy("switch_motor_power", SwitchMotorPower)
        try:
            response = power_switch(state)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def update_imu(self):

    def update_current_position(self):
        self.last_client_update = #todo message timestamp
        self.motor_current_position =

    def update_buttons(self):

    def update_gamestate(self):
        self.penalize_active =

    def publish_motion_state(self):


    cpdef set_state(self, int state):
        """ Updatet den internen Status des MotionServers und publiziert
            ihn zum Clienten
        """
        #Wenn die Motion aus einem State kommt, in dem sie Aufsteht soll der Gyro resettet werden
        #todo kalman scheint nicht mehr benutzt zu werden, löschen? ne wird eigentlich noch bentuzt oder
        if self.state in [
                STATE_FALLEN,
                STATE_FALLING,
                STATE_GETTING_UP,
                STATE_PENALTY,
                STATE_PENALTY_ANIMANTION,
                STATE_STARTUP,
                STATE_BUSY,
                STATE_GETTING_UP_Second]:
            #todo service!
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
            # Remember smoothed gyro values
            # Used for falling detectiont, smooth_gyro is to late but peaks have to be smoothed anyway
            # increasing smoothing -->  later detection
            # decreasing smoothing --> more false positives
            self.smooth_gyro = self.smooth_gyro * 0.9 + self.gyro * 0.1 ###gyro
            self.smooth_accel = self.smooth_accel * 0.9 + self.accel * 0.1 ###accel
            self.not_much_smoothed_gyro = self.not_much_smoothed_gyro * 0.5 + self.gyro * 0.5

            self.update_once()

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

        ###
        ### STATE STARTUP
        ###
        if time.time() - self.startup_time < 1:
            # we're still starting up, do nothing
            rospy.sleep(0.1)
            return

        ###
        ### First run
        ###
        if self.first_run:
            self.first_run = False
            if self.record_active:
                # recording could have already been set by service
                self.set_state(STATE_RECORD)
            else:
                # Play the start up animation and then go to controlable
                self.set_state(STATE_GETTING_UP)
                self.animate(rospy.get_param("/animations/motion/start_up_animation"), STATE_CONTROLABLE)


        ###
        ### Soft off
        ###
        if self.state == STATE_SOFT_OFF:
            # nothing important is happening, get some rest for the CPU
            rospy.sleep(0.05)
            return

        ###
        ### Penalizing
        ###
        if self.penalize_active:
            if self.state == STATE_PENALTY:
                # Already penealized
                # turn of motors and wait
                self.switch_motor_power(False)
                rospy.sleep(0.05)
                # update last_cliente time to prohibit shutdown of motion while beeing penalized
                self.last_client_update = time.time()
                return
            if self.state != STATE_PENALTY_ANIMANTION:
                # We are not yet sitting down, we should start the animation
                rospy.logwarn("Penalized, sitting down!")
                self.set_state(STATE_PENALTY_ANIMANTION)
                #todo service call
                self.animate(
                    rospy.get_param("/animations/motion/penalized"), STATE_PENALTY)
                rospy.logwarn("Motion will not move, I'm penalized")
                return
        elif self.state == STATE_PENALTY:
            # We are not penalized by the gamecontroler anymore, but we are still in STATE_PENALIZED
            # we want to stand up and get into STATE_CONTROLABLE

            # First turn on the motors
            self.switch_motor_power(True)
            # Update the state
            self.set_state(STATE_PENALTY_ANIMANTION)
            # Stand up and get into STATE_CONTROLABLE afterwards
            self.animate("Automatische Walkready", STATE_CONTROLABLE, self.walkready_animation)
            return

        ###
        ### Recording
        ###
        if self.record_active and not self.state == STATE_RECORD:
            # Recording is requested, but we are not already in this state
            if self.state == STATE_SOFT_OFF:
                # If coming from STATE_SOFT_OFF we want to stand up first
                self.animate(
                    rospy.get_param("/animations/motion/walkready"), STATE_RECORD)
            elif self.state not in (
                    STATE_RECORD,
                    STATE_ANIMATION_RUNNING ,
                    STATE_GETTING_UP,
                    STATE_STARTUP,
                    STATE_GETTING_UP_Second):
                # simply set the new state
                self.set_state(STATE_RECORD)
            return
        elif not self.record_active and self.state == STATE_RECORD:
            # Recording finished, go back to normal
            self.set_state(STATE_CONTROLABLE)
            return


        ###
        ### Fall-handling
        ###
        # only do if activated on start
        if self.standupflag:

            ##
            ## Falling detection
            ##
            # check if robot is falling
            falling_pose = self.stand_up_handler.check_falling(self.not_much_smoothed_gyro)
            if falling_pose:
                self.stand_up_handler.set_falling_pose(falling_pose, goal_pose)
                self.set_state(STATE_FALLING)
                return

            ###
            ### Standing up
            ###
            if self.state == STATE_FALLING:
                # maybe the robot is now finished with falling and is lying on the floor
                direction_animation = self.stand_up_handler.check_fallen(self.gyro, self.smooth_gyro, self.robo_angle)
                if direction_animation is not None:
                    self.set_state(STATE_FALLEN)
                    # directly starting to get up. Sending STATE_FALLEN before is still important, e.g. localisation
                    self.set_state(STATE_GETTING_UP)
                    #todo run direction animaiton

        ###
        ### Animation
        ###
        if self.animation_requested:
            if self.state == STATE_ANIMATION_RUNNING:
                # Currently we are running an animation
            elif self.state == STATE_WALKING:
                # We have to stop walking first
            elif  self.state == STATE_CONTROLABLE:
                # we can directly begin to play the animation



        ###
        ### Walking
        ###

        ###
        ### Shuting down?
        ###

    cpdef check_queued_animations(self):
        cdef Animator amator
        if self.next_animation is not None and self.robo_pose is not None:
            if self.state == STATE_SOFT_OFF:
                # Wir setzen den timestamp, um dem Softoff mitzuteilen
                # das wir etwas tu wollen.
                self.last_client_update = time.time()
                # warten bis es Aufgewacht ist, dann erst loslegen
                return
            amator = Animator(self.next_animation, self.robo_pose)
            self.animfunc = amator.playfunc(0.025)
            self.next_animation = None

            if self.state == STATE_CONTROLABLE:
                # Ab jetzt spielen wir die Animation
                self.set_state(STATE_ANIMATION_RUNNING)

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
            # TODO: Use config
            if self.state is STATE_GETTING_UP:
                anim = self.stand_up_handler.get_up(self.set_state)
                if anim:
                    self.animate(anim)
                if self.state is STATE_FALLEN:
                    anim = self.stand_up_handler.check_fallen(self.goal_pose, self.set_state, self.state,
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


    cpdef animate(self, name, post_anim_state=None, dict animation=None):
        """ Spielt eine Animation aus einer Datei ab. Dabei werden Befehle
            von einem IPC Client ignoriert, bis die Animation durchgelaufen
            ist.

            Die Animation wird in dieser Methode geladen und als
            :attr:`next_animation` gespeichert, um bei nächster Gelegenheit
            abgespielt zu werden. Wenn die Animation zuende ist, wird der Status
            auf post_anim_status gesetzt, bei NONE auf STATE_CONTROLABLE
        """
        debug.log("Lade Animation '%s'" % name)
        cdef Animation anim
        try:
            if animation is None or not self.dynamic_animation:
                with open(find_animation(name)) as fp:
                    anim = parse(json.load(fp))
            else:
                anim = parse(animation)
        except IOError:
            debug.warning("Animation '%s' nicht gefunden" % name)
            raise

        self.next_animation = anim
        self.post_anim_state = post_anim_state

    def reconfigure(self, config, level):
        # just pass on to the StandupHandler, as the variables are located there
        self.stand_up_handler.update_reconfigurable_values(config, level)
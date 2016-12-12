# -*- coding: utf8 -*-
import json
import time

import rospy
from bitbots_common.pose.pypose import PyPose as Pose
from dynamic_reconfigure.server import Server
from humanoid_league_msgs.msg import MotionState
from humanoid_league_msgs.msg import Speak
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from bitbots_animation.src.bitbots_animation.animation import parse
from bitbots_motion.src.motion_state_machine import MotionStateMachine, STATE_RECORD, STATE_GETTING_UP, STATE_SOFT_OFF
from .cfg import bitbots_motion_params


class Motion(object):

    def __init__(self, dieflag, standupflag, softoff, softstart):
        rospy.init_node('bitbots_motion', anonymous=False)
        rospy.Subscriber("/IMU", Imu, self.update_imu)
        rospy.Subscriber("/MotorCurrentPosition", JointState, self.update_current_pose)
        rospy.Subscriber("/MotorGoals", JointTrajectory, self.update_goal_position)
        rospy.Subscriber("/pause", bool, self.pause)
        self.joint_goal_publisher = rospy.Publisher('/MotionMotorGoals', JointState, queue_size=10)
        self.state_publisher = rospy.Publisher('/MotionState', JointState, queue_size=10)
        self.speak_publisher = rospy.Publisher('/Speak', Speak, queue_size=10)
        self.dyn_reconf = Server(bitbots_motion_params, self.reconfigure)

        self.dieflag = dieflag
        self.softoff = softoff
        self.softstart = softstart

        self.accel = (0,0,0)
        self.gyro = (0,0,0)
        self.smooth_accel = (0, 0, 0)
        self.smooth_gyro = (0, 0, 0)
        self.not_much_smoothed_gyro = (0, 0, 0)

        self.robo_pose = Pose()
        self.goal_pose = Pose()
        self.startup_time = time.time()
        self.first_run = True

        #todo if record changes, call set_record method
        self.record_active = False # record UI in use #todo set in constructor or smt
        self.animation_requested = False # animation request from animation server

        self.state_machine = MotionStateMachine(standupflag)

        if softstart and not self.record_active:
            rospy.loginfo ("Softstart")
            # time -120 damit softoff wieder anstellen nicht anspringt
            self.last_client_update = rospy.Time.now() - 120
            # das muss hgier schon, sonst geht die src sofort aus,
            # weil er merkt das noch kein update gekommen ist
            self.state_machine.set_state(STATE_SOFT_OFF)
            self.state_machine.switch_motor_power(False)
        else:
            self.last_client_update = time.time()
            self.state_machine.switch_motor_power(True)

        self.update_forever()

    def pause(self, msg):
        self.state_machine.set_penalized(msg)

    def update_imu(self, msg):
        #todo check if this is not switched
        self.gyro = msg.linear_velocity
        self.accel = msg.angular_velocity

        #todo check if this is needed by something else in the software
        #todo make smoothing factors reconfigurable
        # Remember smoothed gyro values
        # Used for falling detectiont, smooth_gyro is to late but peaks have to be smoothed anyway
        # increasing smoothing -->  later detection
        # decreasing smoothing --> more false positives
        self.smooth_gyro = self.smooth_gyro * 0.9 + self.gyro * 0.1  ###gyro
        self.smooth_accel = self.smooth_accel * 0.9 + self.accel * 0.1  ###accel
        self.not_much_smoothed_gyro = self.not_much_smoothed_gyro * 0.5 + self.gyro * 0.5

    def update_current_pose(self, msg):
        self.state_machine.set_last_client_update(msg.header.stamp)
        self.robo_pose.setPositions(msg.positions)
        self.robo_pose.setSpeed(msg.velocities)

    def publish_motion_state(self):
        msg = MotionState()
        msg.state = self.state_machine.get_current_state()
        self.state_publisher.publish(msg)

    def reconfigure(self, config, level):
        # just pass on to the StandupHandler, as the variables are located there
        self.stand_up_handler.update_reconfigurable_values(config, level)

    def update_goal_position(self):
        # we can only handle one point and not a full trajectory
        msg = JointTrajectoryPoint()
        msg.positions = self.goal_pose.get_positions()
        msg.velocities = self.goal_pose.get_speeds()
        traj_msg = JointTrajectory()
        traj_msg.points = []
        traj_msg.points.append(msg)
        traj_msg.header.stamp = rospy.Time.now()
        self.joint_goal_publisher.publish(traj_msg)

    def update_forever(self):
        """ Ruft :func:`update_once` in einer Endlosschleife auf """
        iteration = 0
        duration_avg = 0
        start = rospy.Time.now()

        while True:
            self.update_once()

            # Count to get the update frequency
            iteration += 1
            if iteration < 100:
                continue

            if duration_avg > 0:
                duration_avg = 0.5 * duration_avg + 0.5 * (rospy.Time.now() - start)
            else:
                duration_avg = (rospy.Time.now() - start)

            rospy.loginfo("Updates/Sec %f", iteration / duration_avg)
            iteration = 0
            start = time.time()

    def update_once(self):

        ###
        ### STARTUP
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
                self.state_machine.set_state(STATE_RECORD)
            else:
                # Play the start up animation and then go to controlable
                self.state_machine.set_state(STATE_GETTING_UP)
                self.animate(rospy.get_param("/animations/motion/start_up_animation"), STATE_CONTROLABLE)


        moveable, animation = self.state_machine.evaluate()

        if animation is not None:
            # The state machine requests an animation
            pass #todo

        elif moveable:
            # We can do what animation_node and walking want, in this priority order

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

    def check_queued_animations(self):
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

    def update_goal_pose(self):
        """ Updated :attr:`goal_pose`. Wenn eine Animation läuft, spielt
            die mit hinein. Sonst kommt sie vermutlich vom Client. Außerdem
            werden hier alle anderen Befehle vom Client verarbeitet (walking,
            tracking)
        """

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

    def animate(self, name, post_anim_state=None, animation=None):
        """ Spielt eine Animation aus einer Datei ab. Dabei werden Befehle
            von einem IPC Client ignoriert, bis die Animation durchgelaufen
            ist.

            Die Animation wird in dieser Methode geladen und als
            :attr:`next_animation` gespeichert, um bei nächster Gelegenheit
            abgespielt zu werden. Wenn die Animation zuende ist, wird der Status
            auf post_anim_status gesetzt, bei NONE auf STATE_CONTROLABLE
        """
        rospy.logdebug("Lade Animation '%s'" % name)
        try:
            if animation is None or not self.dynamic_animation:
                with open(find_animation(name)) as fp:
                    anim = parse(json.load(fp))
            else:
                anim = parse(animation)
        except IOError:
            rospy.logwarn("Animation '%s' nicht gefunden" % name)
            raise

        self.next_animation = anim
        self.post_anim_state = post_anim_state

